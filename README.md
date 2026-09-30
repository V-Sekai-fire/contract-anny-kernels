# contract-anny-kernels

The ANNY body model's forward and backward kernels in Lean, emitted through Slang for the in-guest fits.

Split out of `interactor-dress-on` at `310b52e` with its history (`git subtree`). It sits at `2-contract/anny-kernels` in the goal manifest (`contract-manifest-taskweft`), and finds the repositories it builds against as sibling checkouts at their manifest paths. `transport-meshing-pen` builds the guest ELFs (`build.sh`, `tools/build.exs`).
