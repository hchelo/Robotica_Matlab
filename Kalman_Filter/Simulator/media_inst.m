function MedPost = media_inst(NumPost,MapLabel)
MedPost = [];
for k = 1:NumPost
      m = min(find(MapLabel(:,4)==k));
      M = max(find(MapLabel(:,4)==k));
      Mpostes = MapLabel(m:M,1:3); % grouping by characteristic
      MedPst = [mean(Mpostes,1) k]; % getting mean of each group
      MedPost = [MedPost;MedPst];
end
