function [ ] = Comparer( M,Dicp,k,ER,t )
%Comparer Compare point cloud after running ICP to expected point cloud
%   M is the expected result. Dicp is the obtaned point cloud after ICP
% k is the number of iteration, t the time needed to run ICP. ER is the
% error function, corresponding to the sum of the distances between the
% points of the 2 point cloud.

S = repmat(2,numel(M(1,:)),1);
    C = repmat([0,0,255],numel(M(1,:)),1);
    figure
    scatter3(M(1,:),M(2,:),M(3,:),S,C);
    hold on
    S = repmat(2,numel(Dicp(1,:)),1);
    C = repmat([255,0,0],numel(Dicp(1,:)),1);
    scatter3(Dicp(1,:),Dicp(2,:),Dicp(3,:),S,C);
    title('Representation of the Target (Blue) and Transformed Reference (Red) vertices');
    
    
    figure
 plot(0:k,ER,'--x');
 xlabel('iteration#');
 ylabel('d_{RMS}');
 legend('kDtree matching and extrapolation');
 title(['Total elapsed time: ' num2str(t(end),2) ' s']);
 
 
    
end

