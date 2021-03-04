function plotMatches(matches, query_keypoints, database_keypoints)

[~, query_indices, match_indices] = find(matches); % return the nonzero linear indices as well as the index

x_from = query_keypoints(1, query_indices);
x_to = database_keypoints(1, match_indices);
y_from = query_keypoints(2, query_indices);
y_to = database_keypoints(2, match_indices);
plot([x_from; x_to], [y_from; y_to],  'g-', 'Linewidth', 1);

end


