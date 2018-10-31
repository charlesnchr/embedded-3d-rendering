% Implemented by Charles, november 2017

clear
clf

%% Init

resx = 96;
resy = 64;

V = lookat([0 0 5]',[0 0 0]',[0 1 0]');
P = perspective(120,1.5,0.1, 100);

dualcone

c{1} = [1 0 0; 1 0 0; 1 0 0];
c{2} = [0.8 0 0; 0.8 0 0; 0.8 0 0];
c{3} = [0.6 0 0; 0.6 0 0; 0.6 0 0];
c{4} = [0.4 0 0; 0.4 0 0; 0.4 0 0];
c{5} = [0.0 0.2 0.2; 0.0 0.2 0.2; 0.0 0.2 0.2];
c{6} = [0.0 0.4 0.4; 0.0 0.4 0.4; 0.0 0.4 0.4];
c{7} = [0.0 0.6 0.6; 0.0 0.6 0.6; 0.0 0.6 0.6];
c{8} = [0.0 0.8 0.8; 0.0 0.8 0.8; 0.0 0.8 0.8];


%% Vertex shader

for i = 1:length(t)
    for j = 1:3
        temp = (t{i}(j,1:3)'-100)/50;
        rotCoord = rotArbitrary(10*pi/180,[0 1 0]) * temp;
        newcoord = P*V*[rotCoord; 1];
        t{i}(j,1:3) = newcoord(1:3)/newcoord(4);
    end
end

%% Clipping (clip space)


%% Screen space

for i = 1:length(t)
    t{i}(:,1) = round((t{i}(:,1)+1)*1/2*resx + 1);
    t{i}(:,2) = round((t{i}(:,2)+1)*1/2*resy + 1);
    t{i}(:,3) = round((t{i}(:,3)+1)*1/2*80 + 1);
end
max(t{i}(:,3))


%% Gen. EdgeBucket
EdgeBucket = cell(1,length(t)*3);
xarr = zeros(1,length(t)*3);
edge_counter = 0;
% ymax = -inf;

ebidx = 1;
for i = 1:length(t)
    
    for j = 1:3
        eb = struct();
        if j < 3
            X = [t{i}(j,1) t{i}(j+1,1)];
            Y = [t{i}(j,2) t{i}(j+1,2)];
        else
            X = [t{i}(3,1) t{i}(1,1)];
            Y = [t{i}(3,2) t{i}(1,2)];
        end
        
        eb.dx = abs(X(2)-X(1));
        eb.dy = abs(Y(2)-Y(1));
        
        % don't include horizontal lines
        if eb.dy < 1.5*eps,continue, end
        
        [eb.ymin, idx1] = min(Y);
        [eb.ymax, idx2] = max(Y);
        eb.x = X(idx2);
        
        
        eb.s = sign( X(idx2) - X(idx1) );
        eb.t = i;
        eb.sum = 0;
        
        EdgeBucket{ebidx} = eb;
        xarr(ebidx) = eb.x;
        edge_counter = edge_counter + 1;
        ebidx = ebidx + 1;
    end
end


%% Sort EdgeBucket (for performance optimisation)

% [~, sidx_or] = sort(xarr);
% EdgeBucket = EdgeBucket(sidx_or);

%% Scanline loop

fb = zeros(resy,resx,3); % framebuffer
zb = 1000*ones(resy,resx,1); % z buffer

AL = zeros(1,length(t)*3);
AL_counter = 0;

for sy = size(fb,1):-1:1
    for i = 1:edge_counter
        if EdgeBucket{i}.ymax == sy
            AL(AL_counter + 1) = i;
            AL_counter = AL_counter + 1;
        end
    end
    
    if AL_counter == 0, continue, end
    
    % sort AL
    tarr = zeros(1,AL_counter);
    for i = 1:AL_counter
        tarr(i) = EdgeBucket{AL(i)}.t;
    end
    [~, sidx] = sort(tarr);
    AL_sort = AL(sidx);
    
    % iterate AL
    for j = 1:(AL_counter-1)
        
        e1 = EdgeBucket{AL_sort(j)};
        e2 = EdgeBucket{AL_sort(j+1)};
        
        if e1.t ~= e2.t
%             fb(sy,round(e1.x), 2) = 255;
%             fb(sy,round(e2.x), 1) = 255;
%             fprintf('e1 %d, e2 %d \n',e1.t,e2.t);
        else
            if e2.x > e1.x
                x1 = e1.x;
                x2 = e2.x;
            else
                x1 = e2.x;
                x2 = e1.x;
            end
%             rgb = c{e1.t};
            
            c1 = c{e1.t}(1,:);
            c2 = c{e1.t}(2,:);
            c3 = c{e1.t}(3,:);
            
            v1 = t{e1.t}(1,:);
            v2 = t{e1.t}(2,:);
            v3 = t{e1.t}(3,:);
            
            for x = round(x1) : round(x2)
                z = tri_int(v1(3),v2(3),v3(3),v1,v2,v3,[x,sy]);
                
                if z < zb(sy,x)
                    rgb = tri_int(c1,c2,c3,v1,v2,v3,[x,sy]);

                    fb(sy, x, 1) = rgb(1);
                    fb(sy, x, 2) = rgb(2);
                    fb(sy, x, 3) = rgb(3);
                    zb(sy,x) = z;
                end
            end
            
        end
        
    end
    
    % update AL and x
    rmind = [];
    AL_counter_temp = AL_counter;
    AL_counter = 0;
    for i = 1:AL_counter_temp
        e = EdgeBucket{AL(i)};
        if sy-1 >= e.ymin        
            AL(AL_counter + 1) = AL(i);
            AL_counter = AL_counter + 1;
            
            e.x = e.x - e.dx/e.dy * e.s;
            EdgeBucket{AL(i)} = e;
        end
    end    
end


%% Vis.
subplot(1,2,1)
for i = [5 6 7 8 2 4 3 1]
patch(t{i}(:,1),t{i}(:,2),c{i}(1,:));
end
% axis off
axis equal
xlabel('x pixels')
ylabel('y pixels')

subplot(1,2,2)
imshow(fb)
set(gca,'ydir','normal')
axis on
xticks(10:10:100)
xlabel('x pixels')
ylabel('y pixels')

function val = tri_int(f1,f2,f3,v1,v2,v3,p)
    
Wv1 = ( (v2(2)-v3(2))*(p(1)-v3(1)) ...
      + (v3(1)-v2(1))*(p(2)-v3(2)) ) ...
      / ( (v2(2)-v3(2))*(v1(1)-v3(1)) ...
        + (v3(1)-v2(1))*(v1(2)-v3(2)) );
Wv2 = ( (v3(2)-v1(2))*(p(1)-v3(1)) ...
      + (v1(1)-v3(1))*(p(2)-v3(2)) ) ...
      / ( (v2(2)-v3(2))*(v1(1)-v3(1)) ...
        + (v3(1)-v2(1))*(v1(2)-v3(2)) );
Wv3 = 1 - Wv1 - Wv2;

val = f1*Wv1 + f2*Wv2 + f3*Wv3;   


end

function V = lookat(eye,at,up)
    n = (at-eye)/norm(at-eye);
    u = up/norm(up);
    v = cross(n,u)/norm(cross(n,u));
    u = cross(v,n);
%     
    V = zeros(4,4);
    V(1:3,1) = v;
    V(1:3,2) = u;
    V(1:3,3) = -n;
    V(1:4,4) = [-dot(v,eye) -dot(u,eye) dot(n,eye) 1];

end
      
function P = perspective(fovy,aspect,near,far)

tanHalf = tan(fovy*pi/180/2);


P = [1/(aspect*tanHalf) 0 0 0;
     0 1/tanHalf 0 0;
     0 0 -(far+near)/(far-near) -1;
     0 0 -2*far*near/(far-near) 0];
 
end