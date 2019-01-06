function [  ] = Afficher( M,Dicp )
%Afficher Plot the Matrix M as a point cloud
%
    S = repmat(2,numel(M(1,:)),1);
    C = repmat([0,0,255],numel(M(1,:)),1);
    figure
    scatter3(M(1,:),M(2,:),M(3,:),S,C);
    hold on
    S = repmat(2,numel(Dicp(1,:)),1);
    C = repmat([255,0,0],numel(Dicp(1,:)),1);
    scatter3(Dicp(1,:),Dicp(2,:),Dicp(3,:),S,C);
    title('Representation of the Target (Blue) and Reference (Red) vertices');
    

end

