
A language model contains three files:
 - foo.corpus: All sentences the voice recognition system should handle.
 - foo.lm: Language model (generated)
 - foo.dict: Dictionary model (generated)

Both generated files should not be edited manually. They are required by
pocketsphinx and are passed as parameters (see the launch file).

The corpus file is technically not necessary for operation, but it's nice to
keep around.

## Creating a new language model:

See the README in [pocketsphinx](https://github.com/felixduvallet/pocketsphinx)
for instructions on creating your own language model.
