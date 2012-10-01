function term = TerminateCondition(E,x)

    term    = (norm(x(1:2)) < E.disk) | (abs(x(1)) > 1000) | (abs(x(2)) > 1000);

end