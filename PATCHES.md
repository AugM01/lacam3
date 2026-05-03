Fork: https://github.com/AugM01/lacam3

# Patches to lacam3

Upstream: https://github.com/Kei18/lacam3  
Base commit: 1a269b7addc1edc4f1f56b194d307190f5491943 (2025-03-09)

## lacam3/src/instance.cpp
**Instance constructor — scen file format**  
Replaced movingai-format regex (`\d+\t.+\.map\t...\t(x_s)\t(y_s)\t(x_g)\t(y_g)\t...`)
with a compact format (`agent_id\tstart_index\tgoal_index`) using flat vertex indices
instead of (x,y) coordinates. Header line (`map_name N seed timestep`) is skipped
automatically as a non-match.

## lacam3/src/post_processing.cpp
**compress_file_zstd()** — added.  
Compresses a file in-place using zstd, optionally deleting the original.
Used to reduce log file disk usage.

**make_log()** — extended.  
Added logging of LaCAM planner flags, run config, and solution metadata for
compatibility with the FICO-clean experiment runner log parser.

## CMakeLists.txt
**zstd linking** — added.  
Replaced `find_package(zstd)` (which resolves to the Windows MSYS installation
under WSL2) with an explicit link to `/usr/lib/x86_64-linux-gnu/libzstd.so`.