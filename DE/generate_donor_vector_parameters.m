function donor_parameters = generate_donor_vector_parameters(xr1,xr2,xr3,F,parameter_constraints)
    A = xr1.A + F *(xr2.A-xr3.A);
    B = xr1.B + F *(xr2.B-xr3.B);
    radius = xr1.radius + F *(xr2.radius-xr3.radius);
    alfa = xr1.alfa + F *(xr2.alfa-xr3.alfa);
    Vd = xr1.Vd + F *(xr2.Vd-xr3.Vd);
    lambda = xr1.lambda + F *(xr2.lambda-xr3.lambda);
    donor_parameters=[A,B,radius,alfa,Vd,lambda];
    
    %controllo sui vincoli
    for j=1:length(donor_parameters)
        if(donor_parameters(j)<parameter_constraints(j,1))
            donor_parameters(j)=parameter_constraints(j,1);
        elseif (donor_parameters(j)>parameter_constraints(j,2))
            donor_parameters(j)=parameter_constraints(j,2);
        end

    end
    
end

