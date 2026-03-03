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
