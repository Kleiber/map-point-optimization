clear all;
close all;
 
fileID = fopen('DatasetTrajectory.txt');
dataset = textscan(fileID,'%s %s %s');
fclose(fileID);

size = 1;

for j=1:size    
   
    close all;
    
    %MOSTRAR TEST QUE ESTA SIENDO EJECUTADA
    run = ['dataset ' int2str(j) ' Creating trajectory...  [dataset : ' dataset{1}{j} ' ' dataset{2}{j} ' ' dataset{3}{j} ']'];
    display(run);

    %CARGAMOS LOS DATOS DE LAS TRAJECTORIAS
    data = load(dataset{1}{j});
    data_ORB = load(dataset{2}{j});
    data_FUS = load(dataset{3}{j});

    %DEFINIMOS LA CURVA DE LAS TRAJECTORIAS
    curve = animatedline('Color','r','LineWidth',2);
    curve_ORB = animatedline('Color','g','LineWidth',2);
    curve_FUS = animatedline('Color','b','LineWidth',2);

    %CONFIGURAMOS COLUMNAS DE LOS DATOS
    colSize = 1;
    colX = 2;
    colY = 3;
    colZ = 4;

    %CONFIGURAMOS TAMANO DE LA VENTANA DEL DIBUJO
    width = 40;
    height = 25;

    %CALCULAMOS LOS PARAMETROS PARA UN CORRECTO DIBUJO
    padding = 0.3;

    minX = min(data_ORB(:,colX)) - padding;
    minY = min(data_ORB(:,colY)) - padding;
    minZ = min(data_ORB(:,colZ)) - padding;
    maxX = max(data_ORB(:,colX)) + padding;
    maxY = max(data_ORB(:,colY)) + padding;
    maxZ = max(data_ORB(:,colZ)) + padding;

    %CONFIGURAMOS LOS RANGOS DEL GRID PARA HACER EL DIBUJO
    set(gca,'XLim',[minX maxX],'YLim',[minY maxY],'ZLim',[minZ maxZ]);
    view(height, width);
    hold on;
    grid on;
    %axis on;
    %title('Trajectory')
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    legend('ground truth', 'ORB-SLAM', 'FUSION');
    set(gcf, 'Units', 'normalized', 'OuterPosition', [0 0 1 1]);
    
    %CONTADOR DE LOS FRAMES DEL VIDEO
    frames = 1;

    %CONFIGURAMOS LA TRASLACION AL ORIGEN Y EL FACTOR DE ESCALA
    origenX = data(colSize,colX);
    origenY = data(colSize,colY);
    origenZ = data(colSize,colZ);

    scalaFactor = 2.757618;
    
    %GRAFICAMOS LA VERDADERA TRAJECTORIA TRALADADA AL ORIGEN Y ESCALADA
    for i=1:length(data(:,colSize))
        %X = data(i,colX);
        %Y = data(i,colY);
        %Z = data(i,colZ);
    
        %addpoints(curve, X, Y, Z);
        %head = scatter3(X, Y, Z, 'filled', 'MarkerFaceColor', 'r', 'MarkerEdgeColor','r');
        %drawnow
        %delete(head);
    end

    %GRAFICAMOS LA TRAJECTORIA GENERADA POR ORB-SLAM
    for i=1: length(data_ORB(:,colSize)) 
        X = data_ORB(i,colX)* scalaFactor;
        Y = data_ORB(i,colY)* scalaFactor;
        Z = data_ORB(i,colZ)* scalaFactor;
        
        addpoints(curve_ORB, data_ORB(i,colX), data_ORB(i,colY),data_ORB(i,colZ));
        head_ORB = scatter3(data_ORB(i,colX), data_ORB(i,colY),data_ORB(i,colZ), 'filled', 'MarkerFaceColor', 'g', 'MarkerEdgeColor','g');
        drawnow
        %F(frames) = getframe(gcf);
        %frames = frames + 1;
        %pause(0.01);
        delete(head_ORB);
    end


    %GRAFICAMOS LA TRAJECTORIA DEL METODO PORPUESTO Y GRABAMOS EL VIDEO
    for i=1: length(data_FUS(:,colSize)) 
        addpoints(curve_FUS, data_FUS(i,colX), data_FUS(i,colY),data_FUS(i,colZ));
        head_FUS = scatter3(data_FUS(i,colX), data_FUS(i,colY),data_FUS(i,colZ), 'filled', 'MarkerFaceColor', 'b', 'MarkerEdgeColor','b');
        drawnow
        %F(frames) = getframe(gcf);
        %frames = frames + 1;
        %pause(0.01);
        
        delete(head_FUS);
    end
    
    hold off;

    %NOMBRE DEL ARCHIVO DE VIDEO LISTO PARA GRABAR
    %filename = dataset{3}{j};
    %filename = [filename(1:findstr(filename,'.') -1) '.avi'];

    %GUARDAMOS EL VIDEO
    %video = VideoWriter(filename,'Uncompressed AVI');
    %video.FrameRate = 60;
    %open(video)
    %writeVideo(video,F);
    %close(video)
    
    %GUARDAMOS COMO JPG
    filename = dataset{3}{j};
    filename = [filename(1:findstr(filename,'.') -1) '.jpg']; 
    saveas(gcf, filename, 'jpg');
end


%READ PLY FILES AND SAVE (POINTCLOUD)
%clear all;
%close all;
%fileID = fopen('lsd_eccv.txt','w');
%[Tri, Pts] = plyread('LSD_eccv_pc.ply','tri');
%for i=1:length(Pts(:,1))
%	fprintf(fileID,'%f %f %f',Pts(i,:));
%	fprintf(fileID, '\n');
%end
%fclose(fileID);

