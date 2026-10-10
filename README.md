# contract-anny-kernels

The ANNY body model's forward and backward kernels in Lean, emitted through Slang for the in-guest fits.

## What it is for

The Lean kernels restate ANNY's data-parallel stages, from blendshapes and the joint regressor to forward kinematics, skinning and the vertex residual, each with its vector-Jacobian product, so the in-guest L-BFGS-B can fit a body to a target mesh. They are emitted as Slang and compiled to C++ for the CPU path and SPIR-V for the GPU path. It finds the repositories it builds against as sibling checkouts at their paths in the goal manifest.

## Build

```sh
kernels/anny/gen.sh
```

## Licence

MIT. See [LICENSE](LICENSE). The `CITATION.cff` files name the licences of the body model and the Lean tree the kernels derive from.
