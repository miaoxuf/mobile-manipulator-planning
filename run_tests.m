function results = run_tests()
%RUN_TESTS Run the unit tests in tests/ and fail loudly on any error.

root = setup_paths();
results = runtests(fullfile(root, 'tests'));
disp(table(results));
if any([results.Failed]) || any([results.Incomplete])
    error('run_tests:failed', '%d of %d tests failed.', ...
          nnz([results.Failed] | [results.Incomplete]), numel(results));
end
fprintf('All %d tests passed.\n', numel(results));
end
