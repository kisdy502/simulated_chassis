# 扫描源码是否被加密：文本文件含 NUL 字节或不可读即异常
$exts = @('.cpp', '.hpp', '.py', '.lua', '.yaml', '.yml', '.xacro', '.urdf', '.xml', '.txt', '.sdf', '.md', '.sh', '.msg', '.srv', '.action')
$bad = @()
$files = Get-ChildItem -Recurse D:\github\simulated_chassis\src, D:\github\simulated_chassis\scripts -File -ErrorAction SilentlyContinue | Where-Object { $exts -contains $_.Extension -and $_.FullName -notmatch '__pycache__' }
$total = 0
foreach ($f in $files) {
    $total++
    try {
        $bytes = [System.IO.File]::ReadAllBytes($f.FullName)
        # 只读前 64KB 判断
        $len = [Math]::Min($bytes.Length, 65536)
        $nul = 0
        for ($i = 0; $i -lt $len; $i++) { if ($bytes[$i] -eq 0) { $nul++ } }
        if ($nul -gt 4) { $bad += "$($f.FullName) [NUL x$nul]" }
    } catch { $bad += "$($f.FullName) [读取失败: $($_.Exception.Message)]" }
}
echo "扫描文件数: $total"
if ($bad.Count -eq 0) { echo "结果: 全部为正常文本，未发现加密特征" } else { echo "异常文件:"; $bad | ForEach-Object { "  $_" } }
