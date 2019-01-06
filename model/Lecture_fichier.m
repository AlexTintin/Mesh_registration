function [ M ] = Lecture_fichier( path,m )
%Lecture_fichier Read an OBJ file.
%   Read obj file indicated in path and keep 1/m vertices from the total
%   number of verticies

format = path(find(ismember(path,'.'),1,'last'):end);

if sum(format ~= '.obj') ~=0
    error('Format error');
end

[file,errmsg] = fopen(path,'r');

if file == -1
    error(errmsg);
end

Msto = ones(3,m);
M = ones(3,m);

for i = 1:14

v = fscanf(file, '%s',1) ;

end

v='v';
k=1;

while v == 'v'

for i = 1:m

v = fscanf(file, '%s',1) ;
 Msto(1,i) = fscanf(file, '%f',1) ;
 Msto(2,i) = fscanf(file, '%f',1) ;
 Msto(3,i) = fscanf(file, '%f',1) ;
end

M(1,k) = Msto(1,m/2);
M(2,k) = Msto(2,m/2);
M(3,k) = Msto(3,m/2);

k=k+1;
end

fclose(file);

end

