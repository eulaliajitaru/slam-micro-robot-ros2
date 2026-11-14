%% 1. Definirea Modelului Matematic (Ingredientele Teoretice)
Transfer_bloc = tf([1], [0.5 1]);    
PID_bloc = pid(3, 1, 0.5);           
sistem_total = feedback(PID_bloc * Transfer_bloc, 1);

%% 2. Import Date și Curățare (Realitatea din Gazebo)
if exist('date_pid.csv', 'file')
    date = readmatrix('date_pid.csv', 'NumHeaderLines', 1);
    timp = date(:, 1);
    eroare = date(:, 2);
    index_valid = isfinite(eroare);
    timp = timp(index_valid);
    eroare = eroare(index_valid);
else
    error('Fișierul date_pid.csv nu a fost găsit!');
end

%% 3. Calcularea Metricilor de Performanță
eroare_absoluta = abs(eroare);
eroare_maxima = max(eroare_absoluta);
eroare_medie = mean(eroare);
rmse = sqrt(mean(eroare.^2));

fprintf('\n--- REZULTATE PERFORMANTA ROBOT REAL ---\n');
fprintf('Eroarea Maxima: %.4f m\n', eroare_maxima);
fprintf('Eroarea Medie:  %.4f m\n', eroare_medie);
fprintf('RMSE:           %.4f m\n', rmse);

%% 4. Analiza de Performanță Teoretică (Step Info)
[y_teorie, t_teorie] = step(sistem_total * 0.5); 
info = stepinfo(y_teorie, t_teorie, 0.5);
fprintf('\n--- INDICATORI TEORETICI (SIMULINK) ---\n');
fprintf('Rise Time:      %.2f s\n', info.RiseTime);
fprintf('Settling Time:  %.2f s\n', info.SettlingTime);
fprintf('Overshoot:      %.2f %%\n', info.Overshoot);

%% 5. Grafic 1: Analiză Statistică
figure('Color', 'w', 'Name', 'Analiza Statistica');
subplot(2,1,1);
plot(timp, eroare, 'b', 'LineWidth', 1.2);
hold on; yline(0, 'r--', 'LineWidth', 2); grid on;
xlim([min(timp)+150, max(timp)]); 
title('Detaliu Eroare în Regim Stabilizat');
xlabel('Timp (s)'); ylabel('Eroare (m)');

subplot(2,1,2);
histogram(eroare, 30, 'FaceColor', [0.4, 0.6, 0.9], 'EdgeColor', 'w');
grid on; title('Distribuția Statistică a Erorii');
xlabel('Eroare (m)'); ylabel('Frecventa');

%% 6. Grafic 2: Validare Teorie vs Realitate (Folosind iesire_sim)
figure('Color', 'w', 'Name', 'Validare Model');
plot(timp - timp(1), eroare + 0.5, 'b', 'LineWidth', 0.8); hold on;

% Rulăm o simulare rapidă cu P=3 (valoarea ta de bază) pentru validare
P = 3; assignin('base', 'P', P);
out = sim('model_robot_pid.slx');

if isprop(out, 'iesire_sim')
    plot(out.tout, out.iesire_sim, 'r', 'LineWidth', 2);
else
    % Backup dacă blocul nu e configurat bine
    plot(t_teorie, y_teorie, 'r--', 'LineWidth', 2);
end

grid on; title('Validarea Modelului: Simulink vs. Robot Real');
xlabel('Timp (s)'); ylabel('Distanta (m)');
legend('Robot Real (Gazebo)', 'Model Teoretic (Simulink)');

%% 10. Analiza de Stabilitate (Bode)
bucla_deschisa = PID_bloc * Transfer_bloc;
figure('Color', 'w', 'Name', 'Analiza de Robustete Bode');
margin(bucla_deschisa); 
grid on;
[Gm, Pm, Wcg, Wcp] = margin(bucla_deschisa);
fprintf('\n--- CONCLUZII STABILITATE REALE ---\n');
fprintf('Marginea de Faza (Pm): %.2f grade\n', Pm);
fprintf('Marginea de Castig (Gm): %.2f dB\n', 20*log10(Gm));

%% 11. Graficul Final: Analiza de Sensibilitate (Varianta Imbatabilă)
valori_P = [1, 3, 10]; 
culori = {'g', 'r', 'b'};
nume_model = 'model_robot_pid';

% Cream figura și păstrăm referința (handler-ul) ei
fig = figure('Color', 'w', 'Name', 'Studiu Comparativ PID');
ax = axes('Parent', fig); % Cream axele explicit
hold(ax, 'on'); % Activăm hold-ul pe acele axe specifice
grid(ax, 'on');

for i = 1:length(valori_P)
    P_val = valori_P(i);
    fprintf('Rulez simularea pentru P = %d...\n', P_val);
    
    % Injectăm variabila P direct în simulare
    simIn = Simulink.SimulationInput(nume_model);
    simIn = simIn.setVariable('P', P_val);
    
    simOut = sim(simIn); 
    
    % Desenăm direct pe axele noastre (ax)
    plot(ax, simOut.tout, simOut.iesire_sim, 'Color', culori{i}, ...
         'LineWidth', 2, 'DisplayName', ['P = ' num2str(P_val)]);
    
    title(ax, ['Se randează... P = ' num2str(P_val)]);
    legend(ax, 'show', 'Location', 'southeast');
    drawnow; % Forțează desenarea imediată
    pause(0.5); % Mică pauză ca să vezi cum se adaugă liniile
end

% Setări finale după ce s-a terminat bucla
yline(ax, 0.5, '--k', 'Target 0.5m', 'LineWidth', 2, 'HandleVisibility', 'off');
title(ax, 'Impactul parametrului P asupra raspunsului sistemului');
xlabel(ax, 'Timp (s)'); ylabel(ax, 'Distanta (m)');
hold(ax, 'off');

fprintf('Gata! Toate cele 3 linii ar trebui să fie vizibile.\n');