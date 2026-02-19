# 🤖 Firmware Embarqué - Robot SRS

Ce dépôt contient le code source bas niveau (C++) pour les microcontrôleurs du robot SRS. Le système est divisé en deux cartes distinctes pour garantir la sécurité matérielle, la séparation des flux de puissance, et la gestion des périphériques.

## 🏗️ Architecture du Système

Le code communique avec un ordinateur central (Raspberry Pi / PC sous ROS 2) via un protocole série JSON non-bloquant à 115200 bauds.

```mermaid
graph LR
    classDef ros fill:#D95F02,stroke:#fff,stroke-width:2px,color:#fff,font-weight:bold;
    classDef mega fill:#00979D,stroke:#fff,stroke-width:2px,color:#fff,font-weight:bold;
    classDef uno fill:#17A2B8,stroke:#fff,stroke-width:2px,color:#fff,font-weight:bold;
    classDef hw fill:#E2E2E2,stroke:#333,stroke-width:1px,color:#333;

    subgraph CERVEAU ["Cerveau (PC / ROS 2 Humble)"]
        direction TB
        PC((Nœuds Python)):::ros
    end

    subgraph MEGA_SYS ["Sous-système Mouvement & Sécurité"]
        direction TB
        MEGA{Arduino MEGA}:::mega
        Drv["4x Drivers DM542"]:::hw
        Mot["4x Moteurs Z, Y, Magasin, Vérin"]:::hw
        FinsCourse["8x Fins de course (Min/Max)"]:::hw
        INA["4x Capteurs Puissance (INA260)"]:::hw
        ALM["3x Retours Alarme Drivers"]:::hw
        
        MEGA -->|STEP/DIR| Drv
        Drv ==>|Puissance| Mot
        FinsCourse -.->|Pullup LOW| MEGA
        INA -.->|Bus I2C| MEGA
        ALM -.->|Interrupt| MEGA
        Drv -.->|Feedback ALM| ALM
    end

    subgraph UNO_SYS ["Sous-système Périphériques"]
        direction TB
        UNO{Arduino UNO}:::uno
        Capteurs["I2C: IMU, Dist Laser, Temp"]:::hw
        Servos["4x Servomoteurs (Pince/Cam)"]:::hw
        Relais["6x Relais (Pompes, EV)"]:::hw
        LED["Ruban NeoPixel (D8)"]:::hw
        
        Capteurs -.->|Bus I2C| UNO
        UNO -->|PWM| Servos
        UNO -->|Digital HIGH/LOW| Relais
        UNO -->|Data 800kHz| LED
    end

    MEGA <==>|USB ttyACM0 - 10Hz| PC
    UNO <==>|USB ttyACM1 - 4Hz| PC