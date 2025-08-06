%% Εργασία Α - Ρομποτική
% Μπαρμπαγιάννος Βασίλειος
% ΑΕΜ 10685

clc, clear, close all;

% Μήκη.
l = 1;
l0 = 0.1;
h = 0.7;

% Γωνίες.
theta1 = 45;
theta2 = 30;

% Μετατροπή σε ακτίνια.
theta1 = deg2rad(theta1);
theta2 = deg2rad(theta2);

% Ομογενείς Μετασχηματισμοί.
% Από πλαίσιο βάσης {0} στο πλαίσιο {d}.
g0d = [1 0 0 0;
       0 1 0 2;
       0 0 1 0;
       0 0 0 1];
% Από πλαίσιο {d} στο πλαίσιο {h}.
gdh = [0 1 0 0.9;
      -1 0 0 0;
       0 0 1 0.7;
       0 0 0 1];
T0h = g0d*gdh; % Από το {0} στο {h}.
% Από πλαίσιο {h} στο πλαίσιο {h1}.
ghh1 = [1 0 0 0;
        0 cos(-theta1) -sin(-theta1) 0;
        0 sin(-theta1) cos(-theta1) 0;
        0 0 0 1];
T0h1 = T0h*ghh1; % Από το {0} στο {h1}.
% Από πλαίσιο {d} στο πλαίσιο {d1}.
gdd1 = [cos(-theta2) -sin(-theta2) 0 0;
        sin(-theta2) cos(-theta2) 0 0;
        0 0 1 0;
        0 0 0 1];
T0d1 = g0d*gdd1; % Από το {0} στο {d1}.
% Από πλαίσιο {d1} στο πλαίσιο {h3}.
gd1h2 = [0 1 0 0.9;
        -1 0 0 0;
         0 0 1 0.7;
         0 0 0 1];
T0h3 = T0d1*gd1h2; % Από το {0} στο {h3}.
% Από πλαίσιο {h3} στο πλαίσιο {h2}.
gh3h2 = [1 0 0 0;
        0 cos(-theta1) -sin(-theta1) 0;
        0 sin(-theta1) cos(-theta1) 0;
        0 0 0 1];
T0h2 = T0h3*gh3h2; % Από το {0} στο {h2}.

disp('Ομογενής μετασχηματισμός από πλαίσιο βάσης {0} σε πλαίσιο {d}:');
disp(g0d);

disp('Ομογενής μετασχηματισμός από πλαίσιο {d} σε πλαίσιο {h}:');
disp(gdh);

disp('Ομογενής μετασχηματισμός από πλαίσιο {h} σε πλαίσιο {h1}:');
disp(ghh1);

disp('Ομογενής μετασχηματισμός από πλαίσιο {d} σε πλαίσιο {d1}:');
disp(gdd1);

disp('Ομογενής μετασχηματισμός από πλαίσιο {d1} σε πλαίσιο {h2}:');
disp(gd1h2);

disp('Παρατηρώ ότι οι ομογενείς μετασχηματισμοί gd1h2 και gdh είναι ίδιοι, όπως τονίζεται και στην εκφώνηση!');
result = isequal(gdh, gd1h2);
disp(['gd1h2 == gdh: ', mat2str(result)]);

% Τα πλαίσιο στον 3D χώρο, όπως απεικονίζονται στο σχήμα 1.
figure(1);
trplot(eye(4), 'frame', '0', 'color', 'k'); hold on;
trplot(g0d, 'frame', 'd', 'color', 'b');
trplot(T0h, 'frame', 'h', 'color', 'r');
trplot(T0h1, 'frame', 'h1', 'color', 'c');
trplot(T0d1, 'frame', 'd1', 'color', 'b');
trplot(T0h2, 'frame', 'h2', 'color', 'c');
trplot(T0h3, 'frame', 'h3', 'color', 'r');
axis equal;
grid on;
view(3);

T_total = 5; % 5 sec.
N = 100; % Θέσετε όσα βήματα θέλετε.
t = linspace(0, T_total, N);
tau = t / T_total;

% Quintic time scaling: 5ου βαθμού.
q  = 10*tau.^3 - 15*tau.^4 + 6*tau.^5; % θέση.
qd = (30*tau.^2 - 60*tau.^3 + 30*tau.^4) / T_total; % ταχύτητα.
qdd = (60*tau - 180*tau.^2 + 120*tau.^3) / T_total; % επιτάχυνση.

% Οι πόζες μου.
poses = {T0h, T0h1, T0h2, T0h3};
colors = {'r', 'c', 'c', 'r'}; % Χρωματάκι για να χτυπάει καλύτερα στο μάτι.
segments = length(poses) - 1;
N_seg = floor(N / segments);

figure(2);
axis equal; grid on; view(3);
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Τροχιά προσανατολισμού')
hold on;

for i = 1:4
    trplot(poses{i}, 'frame', ['P' num2str(i)], 'color', colors{i});
end

% Αποθήκευση θέσεων για σχεδίαση διαδρομής.
all_positions = [];

% Υπολογισμός τροχιάς και animation.
for seg = 1:segments
    T_start = poses{seg};
    T_end   = poses{seg+1};

    % Θέσεις και quaternions.
    p0 = transl(T_start);
    p1 = transl(T_end);
    q0 = UnitQuaternion(T_start);
    q1 = UnitQuaternion(T_end);

    % Quintic time scaling ανά τμήμα.
    t_local = linspace(0, 1, N_seg);
    s_local = 10*t_local.^3 - 15*t_local.^4 + 6*t_local.^5;

    for i = 1:N_seg
        % Παρεμβολή θέσης και προσανατολισμού.
        p_interp = (1 - s_local(i)) * p0 + s_local(i) * p1;
        q_interp = q0.interp(q1, s_local(i));
        T_interp = rt2tr(q_interp.R, p_interp);

        trplot(T_interp, 'frame', '', 'color', 'g', 'length', 0.1);
        drawnow;

        % Αποθήκευση θέσης.
        all_positions(end+1, :) = p_interp;
    end
end

% Σχεδίαση τροχιάς θέσης στο χώρο.
figure(3);
plot3(all_positions(:,1), all_positions(:,2), all_positions(:,3), 'b-', 'LineWidth', 2, 'DisplayName', 'θέση');
hold on;
plot3(all_positions(1,1), all_positions(1,2), all_positions(1,3), 'go', 'MarkerSize', 8, 'DisplayName', 'Start');
plot3(all_positions(end,1), all_positions(end,2), all_positions(end,3), 'ro', 'MarkerSize', 8, 'DisplayName', 'End');
legend;
grid on; view(3);
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Διαδρομή θέσης στο χώρο.')

% Σχεδίαση q(t).
figure(4);
plot(t, q, 'b-', 'LineWidth', 2);
title('Τροχιά θέσης');
xlabel('t (s)');
ylabel('q(t)');
legend('q(t)');
grid on;

% Σχεδίαση dq(t).
figure(5);
plot(t, qd, 'r-', 'LineWidth', 2);
title('Ταχύτητα');
xlabel('t (s)');
ylabel('dq/dt');
legend('dq/dt');
grid on;

% Σχεδίαση ddq(t).
figure(6);
plot(t, qdd, 'k-', 'LineWidth', 2);
title('Επιτάχυνση');
xlabel('t (s)');
ylabel('d^2q/dt^2');
legend('d^2q/dt^2');
grid on;

%% Εργασία Β - Ρομποτική
% Μπαρμπαγιάννος Βασίλειος
% ΑΕΜ: 10685

close all;

ur10 = ur10robot();

q0 = [-1.7752, -1.1823, 0.9674, 0.2149, 1.3664, 1.5708]; % Από εκφώνηση.

% Από πλαίσιο {e} στο πλαίσιο {h}.
g_he = [0 0 -1 0.1;
        0 1  0 0.1;
        1 0  0 0;
        0 0  0 1];

% Οι τέσσερις πόζες πόμολου, υπολογισμένες στο ερώτημα Α.
poses = {T0h, T0h1, T0h2, T0h3};

T_total = 5; % 5 sec.
N = 500;
dt = T_total / N;
t = linspace(0, T_total, N);

% Quintic time scaling.
s = 10*(t/T_total).^3 - 15*(t/T_total).^4 + 6*(t/T_total).^5;

q = zeros(N,6); % θέσεις αρθρώσεων.
qd = zeros(N,6); % ταχύτητες αρθρώσεων.
T_0h = zeros(4,4,N); % τροχιά του πόμολου.
T_0e = zeros(4,4,N); % Τροχιά του εργαλείου.
sxetikh_poza = zeros(4,4,N); % σχετική θέση και προσανατολισμός του άκρου του ρομπότ ως προς το πόμολο.
theseis_akrou_e = zeros(N,3); % αποθηκεύει τη θέση του άκρου του βραχίονα κάθε χρονική στιγμή.
prosanatolismos_akrou_e = zeros(N,4); % αποθηκεύει τον προσανατολισμό του άκρου του βραχίονα σε quaternions.

q(1,:) = q0;

% Υπολογισμός τροχιάς πόμολου και του άκρου του βραχίονα.
segment_len = floor(N/3); % 3 ενδιάμεσες κινήσεις.
for seg = 1:3
    idx_start = (seg-1)*segment_len + 1;
    idx_end = seg*segment_len;

    T_start = poses{seg};
    T_end = poses{seg+1};

    % Θέσεις και quaternions.
    p0 = transl(T_start);
    p1 = transl(T_end);
    q0_q = UnitQuaternion(T_start);
    q1_q = UnitQuaternion(T_end);

    for i = idx_start:idx_end
        si = s(i - idx_start + 1); % Είναι το s(t) τη χρονική στιγμή i.
        p_interp = (1 - si)*p0 + si*p1; % Γραμμική παρεμβολή θέσης.
        q_interp = q0_q.interp(q1_q, si); % SLERP προσανατολισμού.
        T_0h(:,:,i) = rt2tr(q_interp.R, p_interp); % Συνθέτει τον ομογενή μτσχμ από θέσης + προσανατολισμό.

        % Πόζα του άκρου e.
        T_0e(:,:,i) = T_0h(:,:,i) * g_he;

        % Θέση & προσανατολισμός εργαλείου.
        theseis_akrou_e(i,:) = transl(T_0e(:,:,i))';
        q_e = UnitQuaternion(T_0e(:,:,i));
        prosanatolismos_akrou_e(i,:) = q_e.double;
    end
end

% Τελευταίο σημείο.
T_0h(:,:,N) = poses{end};
T_0e(:,:,N) = T_0h(:,:,N) * g_he;
theseis_akrou_e(N,:) = transl(T_0e(:,:,N))';
prosanatolismos_akrou_e(N,:) = UnitQuaternion(T_0e(:,:,N)).double;

% Αυτό το for loop εκτελεί αυτό που έχω σε κουτάκι στην αναφορά μου, σελ. 8-9.
for i = 1:N-1
    T_now = T_0e(:,:,i); % τρέχουσα πόζα g(t).
    T_next = T_0e(:,:,i+1); % επόμενη πόζα g(t+Δt).

    % Ταχύτητα σώματος/Συστροφή.
    T_delta = T_now \ T_next;
    Vb_mat = logm(T_delta) / dt; % αντισυμμετρικός πίναλας.
    Vb = [Vb_mat(1:3,4); Vb_mat(3,2); Vb_mat(1,3); Vb_mat(2,1)]; % διάνυσμα συστροφής 6x1.

    % Ιακωβιανή στο πλαίσιο βάσης, από την έτοιμη συνάρτηση που δίνεται
    % στην εκφώνηση της εργασίας.
    J = ur10.jacob0(q(i,:));

    % Ταχύτητα αρθρώσεων, σύμφωνα με τον τύπου της θεωρίας.
    qd(i,:) = pinv(J) * Vb;

    % Ολοκλήρωση κατά Euler.
    q(i+1,:) = q(i,:) + qd(i,:) * dt;
end

% Έλεγχος σχετικής πόζας εργαλείου ως προς πόμολο.
for i = 1:N
    sxetikh_poza(:,:,i) = inv(T_0h(:,:,i)) * T_0e(:,:,i);
end

% Plotting της τροχιάς του άκρου του ρομπότ σε 3D.
figure(7);
plot3(theseis_akrou_e(:,1), theseis_akrou_e(:,2), theseis_akrou_e(:,3), 'LineWidth', 2);
title('Τροχιά Θέσης Άκρου Ρομπότ');
xlabel('X'); ylabel('Y'); zlabel('Z');
grid on;
axis equal;

% Θέσεις και ταχύτητες των αρθρώσεων.
figure(8);
subplot(2,1,1);
plot(t, q);
title('Γωνίες Αρθρώσεων');
xlabel('t (s)'); ylabel('q (rad)');
grid on;
subplot(2,1,2);
plot(t, qd);
title('Ταχύτητες Αρθρώσεων');
xlabel('t (s)');
ylabel('dq/dt(rad/s)');
grid on;

% Προσανατολισμός του άκρου του βραχίονα.
figure(9);
plot(t, prosanatolismos_akrou_e);
title('Τροχιά Προσανατολισμού');
xlabel('t (s)');
ylabel('quaternions');
legend('w','x','y','z');
grid on;

% Μετατόπιση του άκρου ως προς πόμολο.
figure(10);
for i = 1:3
    subplot(3,1,i);
    plot(t, squeeze(sxetikh_poza(i,4,:))); % Το squeeze κάνει τον πίνακα 4x4xN σε Nx1 για να μπορέσω να κάνω plot!
    title(['Μετατόπιση άκρου ως προς πόμολο στον άξονα ', char('X'+i-1)]);
    ylabel('m'); xlabel('t (s)'); grid on;
end

% Animation της κίνησης του ρομπότ.
figure(11);
ur10.plot(q(1,:));
for i = 1:10:N
    ur10.plot(q(i,:));
    title(['t = ', num2str(t(i),'%.2f'), ' s']);
    drawnow;
end
