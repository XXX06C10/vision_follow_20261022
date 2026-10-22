param([ValidateSet('tb_full','tb_detector','tb_camera','tb_motion','tb_center','tb_large','tb_follow','tb_plant','tb_loss','tb_rgb')][string]$Testbench='tb_full')
$ErrorActionPreference = 'Stop'
Push-Location $PSScriptRoot
try {
    python compose.py
    if ($LASTEXITCODE -ne 0) { throw 'Source composition failed' }
    $iverilogPath = (Get-Command iverilog -ErrorAction Stop).Source
    $vvpPath = (Get-Command vvp -ErrorAction Stop).Source
    $ivlBase = Join-Path (Split-Path (Split-Path $iverilogPath -Parent) -Parent) 'lib/ivl'
    $ivlPath = Join-Path $ivlBase 'ivl.exe'
    if (-not (Test-Path -LiteralPath $ivlPath)) { throw "Icarus compiler engine missing: $ivlPath" }
    New-Item -ItemType Directory -Force -Path sim/results | Out-Null

    $inputs=@('rtl/vision_monitor_all.v', "sim/$Testbench.v")
    if($Testbench -in @('tb_detector','tb_rgb')){$inputs+='user_source/hdl_source/steel_ball_stream_bbox.v'}
    if($Testbench -eq 'tb_rgb'){$inputs+='user_source/hdl_source/steel_ball_detect.v'}
    if($Testbench -eq 'tb_camera'){
        foreach($source in @('uicfgcs500.v','uics500reg.v','uics500regAE.v','SC500GainTbl.v','uii2c.v')){
            $inputs+='user_source/hdl_source/uics500_cfg/'+$source
        }
    }
    $sourceParts = foreach ($sourcePath in $inputs) {
        $content=Get-Content -LiteralPath $sourcePath -Raw
        # TD accepts this reference implicit wire; declare it before use for Icarus.
        # No logic or register sequence changes in this simulation-only adapter.
        if($sourcePath -like '*/uicfgcs500.v'){$content=$content.Replace('wire ic_busy;', 'wire iic_busy;')}
        # This board uses the real RGB mask/overlay plus stream_bbox; the old
        # in-module CCL diagnostic engine has no connected outputs and TD trims
        # it. Exclude only that unobserved tail from the RGB simulation adapter.
        if($Testbench -eq 'tb_rgb' -and $sourcePath -like '*/steel_ball_detect.v'){
            $cut=$content.IndexOf('    // Synchronous BRAM FIFO')
            if($cut -lt 0){throw 'RGB adapter boundary not found'}
            $content=$content.Substring(0,$cut)+[Environment]::NewLine+'endmodule'+[Environment]::NewLine
        }
        '`line 1 "' + $sourcePath + '" 0' + [Environment]::NewLine + $content
    }
    [System.IO.File]::WriteAllText(
        (Join-Path $PSScriptRoot "sim/results/$Testbench.combined.v"),
        [string]::Join([Environment]::NewLine, $sourceParts),
        (New-Object System.Text.UTF8Encoding($false)))

    $configLines = @("root:$Testbench", "basedir:$ivlBase", 'generation:2012', 'generation:xtypes', 'generation:icarus-misc', "out:sim/results/$Testbench.vvp", 'iwidth:32', 'widthcap:65536')
    foreach ($moduleName in @('system', 'v2005_math', 'v2009', 'va_math')) {
        $configLines += 'module:' + (Join-Path $ivlBase "$moduleName.vpi")
    }
    [System.IO.File]::WriteAllText(
        (Join-Path $PSScriptRoot "sim/results/$Testbench.config"),
        ([string]::Join([Environment]::NewLine, $configLines) + [Environment]::NewLine),
        (New-Object System.Text.UTF8Encoding($false)))

    & $ivlPath -C "sim/results/$Testbench.config" -C (Join-Path $ivlBase 'vvp.conf') -- "sim/results/$Testbench.combined.v"
    if ($LASTEXITCODE -ne 0) { throw 'Compilation failed' }
    & $vvpPath "sim/results/$Testbench.vvp" | Tee-Object -FilePath "sim/results/$Testbench.log"
    if ($LASTEXITCODE -ne 0) { throw 'Simulation failed' }
} finally { Pop-Location }
