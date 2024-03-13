%%
[sps,T] = icm_20948(200);
X=(1:length(T))'; figure; plot(X,T,X,X); figure; plot(X,T-X,'*');
Terr=T-X;
%%
Nstds = ceil(length(Terr)-59);
stds = zeros(Nstds,1);
for i=1:Nstds
  stds(i) = std(Terr(i:i+59));
end
figure; plot(stds,'*');
%%
% figure; plot(diff(stds));
ddiffs = diff(diff(stds)>=0)>0;
setl = find(ddiffs,1)+2;
figure;
plot(X(1:setl-1),Terr(1:setl-1),'*',X(setl:end),Terr(setl:end),'+');
title(sprintf('setl=%d noise=%.3f, 2\\sigma', setl, 2*stds(setl)));
%%
figure; plot(diff(stds)>=0,'*');
%%
figure; plot(stds,'*');

