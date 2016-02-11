function msg = run_in_caller(statement)
% evaluates 'statement' in the caller's workspace and returns an empty
% string if execution completed without errors. otherwise returns the error
% message

try
    if ischar(statement)
        evalin('caller', statement);
    else
        % statement is a function handle
        feval(statement);
    end
	msg = '';
catch
	LE = lasterror;
	msg = LE.message;
end
