function data = readPly(fname)

data = plyread(fname);
if isfield(data.vertex,'image_x') && isfield(data.vertex,'image_y')
   data = [data.vertex.x data.vertex.y data.vertex.z data.vertex.image_x data.vertex.image_y];
else
   data = [data.vertex.x data.vertex.y data.vertex.z];
end

