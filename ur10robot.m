% Δημιουργεί και επιστρέφει ένα μοντέλο ρομπότ τύπου UR10 (Universal Robots).
function ur10 = ur10robot()

    ur10 = mdl_ur10(); % Καλεί αυτή τη συνάρτηση για να δημιουργήσει το μοντέλο του ρομπότ.
    ur10.base.t = [1;1;0]; % Μετά, μετατοπίζει τη θέση του πλαισίου βάσης του κατά αυτό το διάνυσμα.

end
% Αυτή η συνάρτηση κατασκευάζει το ρομποτικό βραχίονα UR10 ως σειριακό σύνδεσμο (Serial Link).
function r = mdl_ur10()
    
    deg = pi/180;
    
    % Ορισμοί των παραμέτρων DH για τους 6 συνδέσμους του UR10.
    % robot length values (metres)
    a = [0, -0.612, -0.5723, 0, 0, 0]'; % απόσταση μεταξύ των αρθρώσεων στον x-άξονα.
    d = [0.1273, 0, 0, 0.163941, 0.1157, 0.0922]'; % μετατόπιση κατά τον z-άξονα.
    alpha = [1.570796327, 0, 0, 1.570796327, -1.570796327, 0]'; % περιστροφή γύρω από τον x-άξονα.
    theta = zeros(6,1); % η γωνία της άρθρωσης (προεπιλεγμένα μηδέν).
    DH = [theta d a alpha];
    
    % Ορίζονται οι μάζες και τα κέντρα μάζας κάθε άρθρωσης του ρομπότ.
    mass = [7.1, 12.7, 4.27, 2.000, 2.000, 0.365];
    center_of_mass = [
        0.021, 0, 0.027
        0.38, 0, 0.158
        0.24, 0, 0.068
        0.0, 0.007, 0.018
        0.0, 0.007, 0.018
        0, 0, -0.026  ];
    
    % and build a serial link manipulator
    
    % offsets from the table on page 4, "Mico" angles are the passed joint
    % angles.  "DH Algo" are the result after adding the joint angle offset.

    % Δημιουργείται το αντικείμενο SerialLink που αναπαριστά το ρομπότ UR10 
    % με βάση τις DH παραμέτρους.
    robot = SerialLink(DH, ...
        'name', 'UR10', 'manufacturer', 'Universal Robotics');
    
    % add the mass data, no inertia available
    % Αντιστοιχίζει σε κάθε άρθρωση τη μάζα και το κέντρο μάζας.
    links = robot.links;
    for i=1:6
        links(i).m = mass(i);
        links(i).r = center_of_mass(i,:);
    end

    
    % place the variables into the global workspace
    
    if nargin == 1 % Ανάθεση μεταβλητών στον caller.
        r = robot;
    elseif nargin == 0 % Αν η συνάρτηση καλεστεί χωρίς παραμέτρους (όπως εδώ), τότε:
        % Δημιουργεί τις μεταβλητές ur10, qz και qr στο workspace του χρήστη:
        assignin('caller', 'ur10', robot);
        assignin('caller', 'qz', [0 0 0 0 0 0]); % qz oυδέτερη θέση (όλες οι γωνίες 0).
        assignin('caller', 'qr', [180 0 0 0 90 0]*deg); % qr κάποια προκαθορισμένη στάση με 
        % γωνίες σε μοίρες (κάθετη θέση όπως στο σχήμα 2 της τεκμηρίωσης)
    end
    r = robot; % Επιστρέφεται το αντικείμενο ur10 το οποίο είναι ένα μοντέλο 
    % 6-DOF του ρομπότ UR10, έτοιμο για χρήση σε προσομοίωση, κινηματική, σχεδιασμό τροχιάς, κ.λπ.

end