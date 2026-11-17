# Provenance

This is MeshBench's own work: a model of a Semtech SX1262, written against the
published datasheet and validated against RadioLib's unmodified driver. It was
extracted from `MeshBench/meshcore-native`, where it lived as
`variants/host/VirtualSX1262.{h,cpp}` and was already MIT.

**Nothing here links MeshCore, RadioLib, or any third-party code.** The model
depends on the C++ standard library and nothing else, which is what lets its
hosts differ so widely in licence:

| host | how it links | its licence |
|---|---|---|
| `MeshBench/meshcore-native` | the C++ class, in process | MIT |
| `MeshBench/qemu` | this ABI, `extern "C"` | GPLv2 |
| `MeshBench/meshbench` | this ABI, via cgo | GPL-3.0-or-later |
| Renode's SX1262 peripheral | this ABI, via `DllImport` | MIT |

MIT is deliberate rather than inherited: a GPLv2 host and a GPL-3.0 host cannot
both link the same copyleft library, and both of those hosts are real.
