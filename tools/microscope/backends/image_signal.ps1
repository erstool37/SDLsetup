param(
    [Parameter(Mandatory = $true)]
    [string]$Path,
    [int]$Step = 64
)

$ErrorActionPreference = "Stop"

try {
    Add-Type -AssemblyName System.Drawing

    if (-not (Test-Path -LiteralPath $Path)) {
        throw "image file not found: $Path"
    }
    if ($Step -lt 1) {
        $Step = 64
    }

    $image = [System.Drawing.Image]::FromFile($Path)
    $bitmap = New-Object System.Drawing.Bitmap $image
    try {
        $sum = 0.0
        $sumSq = 0.0
        $min = 255.0
        $max = 0.0
        $count = 0

        for ($y = 0; $y -lt $bitmap.Height; $y += $Step) {
            for ($x = 0; $x -lt $bitmap.Width; $x += $Step) {
                $pixel = $bitmap.GetPixel($x, $y)
                $value = ($pixel.R + $pixel.G + $pixel.B) / 3.0
                $sum += $value
                $sumSq += ($value * $value)
                if ($value -lt $min) {
                    $min = $value
                }
                if ($value -gt $max) {
                    $max = $value
                }
                $count += 1
            }
        }

        if ($count -eq 0) {
            throw "no pixels sampled"
        }

        $mean = $sum / $count
        $variance = [math]::Max(0.0, ($sumSq / $count) - ($mean * $mean))

        [ordered]@{
            ok = $true
            path = $Path
            width = $bitmap.Width
            height = $bitmap.Height
            sample_step = $Step
            sample_count = $count
            mean = [math]::Round($mean, 3)
            stddev = [math]::Round([math]::Sqrt($variance), 3)
            min = [math]::Round($min, 3)
            max = [math]::Round($max, 3)
        } | ConvertTo-Json -Depth 4
    } finally {
        $bitmap.Dispose()
        $image.Dispose()
    }
} catch {
    [ordered]@{
        ok = $false
        path = $Path
        error = $_.Exception.Message
    } | ConvertTo-Json -Depth 4
    exit 1
}
