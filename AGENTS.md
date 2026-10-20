# Agent Guide

- Read [`README.md`](README.md) and the relevant source and tests before
  changing code. Public headers are under `include/openloc/`; implementation
  and private candidate headers are under `src/`. Read
  [`docs/principles.md`](docs/principles.md) for functional goals,
  [`docs/architecture.md`](docs/architecture.md) for boundaries, and
  [`docs/roadmap.md`](docs/roadmap.md) for status and evidence.
- Keep changes focused, preserve existing work, and do not move host-runtime
  responsibilities into this middleware-independent library.
- Use the package's verified test targets in [`README.md`](README.md); test
  results do not establish vehicle accuracy or production readiness.
- Keep temporary investigations under [`.agents/notes/`](.agents/notes/).
  Workspace-level durable knowledge belongs to
  [`wheelos-service/context/`](../../wheelos-service/context/README.md).
- See the [skill index](.agents/skills/README.md) and
  [knowledge index](.agents/knowledge/README.md).
