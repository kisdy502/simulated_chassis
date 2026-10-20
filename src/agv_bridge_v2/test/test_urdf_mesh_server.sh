#!/usr/bin/env bash
# Isolated HTTP contract test. Never touches an existing robot or port 8090.
set -euo pipefail
server=$1
temporary=$(mktemp -d)
server_pid=
cleanup() {
    if [[ -n "$server_pid" ]]; then
        kill -TERM "$server_pid" 2>/dev/null || true
        wait "$server_pid" 2>/dev/null || true
    fi
    rm -rf -- "$temporary"
}
trap cleanup EXIT
mkdir -p "$temporary/prefix/share/ament_index/resource_index/packages" "$temporary/assets/meshes"
touch "$temporary/prefix/share/ament_index/resource_index/packages/test_mesh"
ln -s "$temporary/assets" "$temporary/prefix/share/test_mesh"
printf 'mesh bytes\n' > "$temporary/assets/meshes/test file.STL"
printf '<COLLADA/>\n' > "$temporary/assets/meshes/model.dae"
dd if=/dev/zero of="$temporary/assets/meshes/large.stl" bs=1048576 count=4 status=none
export AMENT_PREFIX_PATH="$temporary/prefix:${AMENT_PREFIX_PATH:-}"
port=$((20000 + $$ % 30000))
base="http://127.0.0.1:$port"
"$server" --host 127.0.0.1 --port "$port" > "$temporary/server.log" 2>&1 &
server_pid=$!
ready=false
for ((attempt=0; attempt<200; ++attempt)); do
    if curl --noproxy '*' -fsS --max-time 1 "$base/test_mesh/meshes/model.dae" > /dev/null 2>&1; then
        ready=true
        break
    fi
    if ! kill -0 "$server_pid" 2>/dev/null; then
        cat "$temporary/server.log"
        exit 1
    fi
    sleep 0.05
done
[[ "$ready" == true ]]

curl --noproxy '*' -fsS -D "$temporary/headers" \
    "$base/test_mesh/meshes/test%20file.STL?version=1" -o "$temporary/body"
cmp "$temporary/body" "$temporary/assets/meshes/test file.STL"
grep -qi '^Access-Control-Allow-Origin: \*' "$temporary/headers"
grep -qi '^Content-Type: model/stl' "$temporary/headers"
curl --noproxy '*' -fsSI "$base/test_mesh/meshes/test%20file.STL" > "$temporary/head"
grep -qi '^Content-Length: 11' "$temporary/head"

for path in / /test_mesh /test_mesh/meshes/ /unknown_package/a.stl \
    /test_mesh/../meshes/model.dae /test_mesh/%2e%2e/etc/passwd \
    /test_mesh/meshes%2f..%2f..%2fsecret /test_mesh/%00.stl \
    /test_mesh/%5c..%5csecret /test_mesh/%ZZ /test_mesh/no.stl; do
    code=$(curl --noproxy '*' --path-as-is -sS -o /dev/null -w '%{http_code}' "$base$path")
    [[ "$code" == 404 ]] || { echo "Unexpected status $code for $path"; exit 1; }
done
[[ $(curl --noproxy '*' -sS -X POST -o /dev/null -w '%{http_code}' "$base/test_mesh/meshes/model.dae") == 405 ]]
[[ $(curl --noproxy '*' -sS -X OPTIONS -D "$temporary/options" -o /dev/null -w '%{http_code}' "$base/test_mesh/meshes/model.dae") == 204 ]]
grep -qi '^Access-Control-Allow-Methods: GET, HEAD, OPTIONS' "$temporary/options"

# Parallel browser asset requests and a binary mesh larger than an HTTP buffer.
clients=()
for ((index=0; index<8; ++index)); do
    curl --noproxy '*' -fsS --max-time 15 "$base/test_mesh/meshes/large.stl" -o "$temporary/large.$index" &
    clients+=("$!")
done
for client in "${clients[@]}"; do wait "$client"; done
for ((index=0; index<8; ++index)); do
    cmp "$temporary/large.$index" "$temporary/assets/meshes/large.stl"
done

# Invalid arguments and a busy port must fail, not pretend startup succeeded.
for invalid in 0 65536 -1 abc 8090abc; do
    if "$server" --port "$invalid" > /dev/null 2>&1; then exit 1; fi
done
if "$server" --host 127.0.0.1 --port "$port" > /dev/null 2>&1; then exit 1; fi
kill -TERM "$server_pid"
wait "$server_pid"
server_pid=
echo 'URDF mesh HTTP contract: PASS (assets, CORS, HEAD, traversal, parallel transfer, CLI, shutdown)'
