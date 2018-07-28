function [ ins_timestamps,ins_poses, ins_quaternions, pitches, states] = readINS( ins_file )

    ins_file_id = fopen(ins_file);
    headers = textscan(ins_file_id, '%s', 15, 'Delimiter',',');
    ins_data = textscan(ins_file_id, ...
      '%u64 %s %f %f %f %f %f %f %s %f %f %f %f %f %f','Delimiter',',');
    fclose(ins_file_id);

    ins_timestamps = ins_data{1};
    states = ins_data{2};

    northings = ins_data{6};
    eastings = ins_data{7};
    downs = ins_data{8};
    rolls = ins_data{13};
    pitches = ins_data{14};
    yaws = ins_data{15};

    ins_poses = cell(1, size(northings, 1));
    ins_quaternions = cell(1, size(northings, 1));

    for i = 1:size(northings, 1)
        ins_poses{i} = SE3MatrixFromComponents(...
                  northings(i), eastings(i), downs(i), rolls(i), pitches(i), yaws(i));
        ins_quaternions{i} = SO3ToQuaternion(ins_poses{i}(1:3,1:3))';
    end

end

