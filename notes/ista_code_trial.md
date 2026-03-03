# ISTA Code Trial (separate branch)

Branch: `ista_code_trial`
Date: 2026-03-03

Requested URL:
- https://visualcomputing.ist.ac.at/code/

## What I found
Public repos linked on that page:
- https://git.ista.ac.at/wojtan-group/yi-lu-chen/frictionfrenzy
- https://git.ista.ac.at/yichen/primal-dual-friction-public
- https://git.ista.ac.at/psynak/superdupertopofixer
- plus two homogenization repos (`lgw-gggg`, `lgw-mmmm`)

## Relevance to this yarn project
The most relevant candidate for future contact/friction work is:
- `frictionfrenzy` (non-smooth Coulomb friction solver for many rigid-body contacts)

But it is not a yarn/FEM strand model, so it does not directly replace the hanging yarn simulation.

## Recommendation
- Keep current yarn simulation stack as primary.
- If needed later, integrate ideas from `frictionfrenzy` only for contact/friction stage.

## MADYPG trial (paper 10.1145/3450626.3459816)
Repository:
- `external/ista_madypg` (official ISTA MADYPG code)

Status:
- Code builds successfully on this machine in non-parallel mode.
- Working build command:
  - `python exec.py -p 0 mesh2yarns 1`
- `-p 0` is required here because upstream code uses legacy TBB task API, while available TBB is oneTBB and causes link failures in parallel mode.

Compatibility fixes applied in local MADYPG clone:
- add missing `<cstdint>` includes in FBX and bitsery headers
- add missing `<limits>` include in bitsery serializer
- link `ZLIB::ZLIB` for bundled FBX utility target

Runtime note:
- If SDL reports `No available video device`, run outside tmux or propagate `DISPLAY` / `WAYLAND_DISPLAY` / `XDG_RUNTIME_DIR` into tmux environment.
