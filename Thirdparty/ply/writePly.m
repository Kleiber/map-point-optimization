function writePly(data, fname, colors)

elements.vertex.x = single(data(:,1));
elements.vertex.y = single(data(:,2));
elements.vertex.z = single(data(:,3));

if nargin == 3
elements.vertex.diffuse_red = colors(:,1);
elements.vertex.diffuse_green = colors(:,2);
elements.vertex.diffuse_blue = colors(:,3);
end

if size(data,2) == 5
elements.vertex.image_x = int32(data(:,4));
elements.vertex.image_y = int32(data(:,5));
end

plywrite(elements,fname);

