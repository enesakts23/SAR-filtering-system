function fuzetakibi()
    % Harita boyutları ve hedefin konumu
    harita_genislik = 100;
    harita_yukseklik = 100;
    hedef_x = 50;
    hedef_y = 50;

    % Simülasyon süresi ve adım sayısı
    sim_suresi = 10; % saniye cinsinden
    adim_sayisi = 100;

    % Füze hızı
    fuzehizi = 10; % birim/saniye

    % Füze başlangıç pozisyonu
    fuzepoz_x = 0;
    fuzepoz_y = 0;

    % Füze izinin saklanacağı dizi
    fuzegiz_yolu = zeros(adim_sayisi, 2);

    % Hedefin takip edildiği süre
    tespitsuresi = 0;

    % GUI figure oluşturulması
    fig = figure('Position', [100, 100, 600, 600]);
    ax = axes('Parent', fig, 'Position', [0.05, 0.3, 0.9, 0.65]);
    radar_sayaci = uicontrol('Style', 'text', 'Position', [250, 50, 100, 30]);

    % Simülasyon döngüsü
    for adim = 1:adim_sayisi
        % Füze hareketi
        fuzepoz_x = fuzepoz_x + fuzehizi * sim_suresi / adim_sayisi;
        fuzepoz_y = fuzepoz_y + fuzehizi * sim_suresi / adim_sayisi;

        % Füze konumunun güncellenmesi
        set(fig, 'CurrentAxes', ax);
        plot(fuzegiz_yolu(:, 1), fuzegiz_yolu(:, 2), 'b--'); % Füze izinin çizimi
        hold on;
        plot(hedef_x, hedef_y, 'ro', 'MarkerSize', 10); % Hedefin çizimi
        plot(fuzepoz_x, fuzepoz_y, 'gx', 'MarkerSize', 10); % Füzenin konumunun çizimi
        hold off;
        xlim([0, harita_genislik]);
        ylim([0, harita_yukseklik]);
        title('Füze Takibi');
        legend('Füze İzi', 'Hedef', 'Füze');
        drawnow;

        % Füzenin izinin saklanması
        fuzegiz_yolu(adim, :) = [fuzepoz_x, fuzepoz_y];

        % Füze radar sisteminde tespit edildi mi kontrolü
        if abs(fuzepoz_x - hedef_x) <= 1 && abs(fuzepoz_y - hedef_y) <= 1
            tespitsuresi = adim * sim_suresi / adim_sayisi;
            set(radar_sayaci, 'String', sprintf('Tespit Süresi: %.2f saniye', tespitsuresi));
        end

        % İterasyon bekleme süresi
        pause(sim_suresi / adim_sayisi);
    end
end
