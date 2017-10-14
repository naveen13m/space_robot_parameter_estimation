Motivation
- A set of inertial parameters which give the desired minimal parameters
are needed to use the minimal parameters for inverse dynamics simulations
with ReDySim. This can be obtained simply by a parameter coupling matrix
which linearly transforms the standard parameter vector to minimal 
parameter vector.

What does this do?
- Computes the symbolic form of parameter coupling matrix which is
convertible to double format also.

How to use it?
- Make an inputs_sym.m file which contains the inertia and geometric 
parameters in symbolic form. Not all need to be symbolic. The symbolic inertia
in the inputs are inertia about the link frame.
    Note: It is suggestible to avoid irrational number along with the symbols
    as much as possible in the inputs file. If it is required, use the symbolic
    variables for the computation and replace them with the required values at 
    the end.
- Set sections 'Config parameters' and 'Symbolic to numeric variables' 
in 'inputs_to_pcm.m' as per the requirements


