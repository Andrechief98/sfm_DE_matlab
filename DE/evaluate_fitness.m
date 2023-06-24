function fitness_value = evaluate_fitness(distance_matrix)
    norm_vector=zeros(length(distance_matrix),1);
    for i=1:length(distance_matrix(:,1))
        norm_vector(i)=norm(distance_matrix(i,1:2));
    end
    fitness_value=sum(norm_vector)/length(norm_vector);

end

