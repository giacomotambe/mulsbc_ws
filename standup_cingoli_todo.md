# Stand-up con assistenza dei cingoli: cose da fare

Obiettivo: quando si preme **△** (stand) il robot si alza con le spalle in **controllo di posizione**, come oggi,
mentre i cingoli ricevono un **comando di velocità di segno opposto** alla velocità delle spalle della stessa gamba.

Vincolo: lo stesso `omni_controller` gira anche sull'**omniquad 12 DoF** (ruote omni/mecanum).
Su quel robot il comportamento **non deve cambiare**: la funzione si attiva solo da YAML ed è spenta di default.

> I riferimenti di riga sono al commit `e8970d7` di `omni_controller`, quello caricato sul robot.
> Sul vostro branch, con il fix della differential IK, possono essere spostati di qualche riga.

---

## 1. La legge di controllo

Riferimento delle spalle, già esistente (cosine interp):

```
q(t) = q_start + ½·(1 − cos(π·t))·(q_target − q_start),     t = elapsed / duration ∈ [0, 1]
```

Velocità della spalla, cioè la derivata del riferimento in forma chiusa (non serve misurarla):

```
q̇(t) = (q_target − q_start) · ½·π·sin(π·t) / duration
```

Comando del cingolo della stessa gamba:

```
ω_cingolo(t) = −k · q̇_spalla(t)          k > 0
```

- Vale 0 a inizio e fine transizione, con il massimo a metà, insieme alla spalla.
- È sempre di segno opposto alla spalla, per costruzione.
- In **rest** q̇ cambia segno, quindi cambia anche il cingolo senza codice in più.
- Il segno è lo stesso per tutte e 4 le gambe, in coordinate motore (verificato con `getup_traq.py`, vedi §5).
- Valore di partenza: **k ≈ 7.4**, ricavato da `getup_traq.py`: 1.79 rad/s di ruota contro 1.2 rad in 5 s di spalla.
  Picco previsto ≈ 2.8 rad/s a metà salita con `stand_duration = 5 s`.

---

## 2. File da modificare (repo `mulsbc_ws`, sul robot)

### 2.1 `src/omni_controller/include/omni_controller/omni_controller.hpp`

- [ ] Aggiungere i membri:
  - `std::string transition_wheels_mode_;` (`"lock"` | `"assist"`)
  - `double transition_wheels_gain_;` (k)
  - `std::map<std::string, std::string> wheel_to_hfe_;` (es. `LF_WHEEL → LF_HFE`)
- [ ] Dichiarare l'helper:
  - `void write_transition_wheel_commands(double t, double duration);`

### 2.2 `src/omni_controller/src/omni_controller.cpp`

#### `on_init()` (~riga 67, sezione "Transitions")
- [ ] Dichiarare i nuovi parametri con default che **non cambiano niente**:
  ```cpp
  auto_declare<std::string>("transition_wheels.mode", "lock");
  auto_declare<double>("transition_wheels.gain", 0.0);
  ```

#### `on_configure()` (~riga 250, sezione "Transition configuration")
- [ ] Leggere `transition_wheels.mode` e `transition_wheels.gain`.
- [ ] Validare:
  - `mode` ∈ {`lock`, `assist`}, altrimenti `CallbackReturn::ERROR`;
  - se `assist` e `gain <= 0`: warning e ritorno a `lock`;
  - se `assist` ma `!has_wheels_ || !has_legs_ || !has_transitions_`: warning e ritorno a `lock`.
- [ ] Costruire `wheel_to_hfe_`: per ogni ruota in `wheel_joints_` prendere il prefisso prima del primo `_`
  (`LF`, `LH`, `RF`, `RH`) e cercare in `joints_` il giunto `<prefisso>_HFE` (usare `is_hfe_joint()`).
  - Se una ruota non ha la spalla corrispondente: warning e ritorno a `lock` (meglio che lasciare il cingolo fermo su una sola gamba).
- [ ] Loggare la configurazione scelta (`RCLCPP_INFO`: mode, gain, mappatura ruota → spalla).

#### `update_transition()` (riga 1191)
- [ ] Sostituire il blocco `// Lock wheels during transition` (righe ~1262–1273) con:
  ```cpp
  if (has_wheels_)
      write_transition_wheel_commands(t, duration);
  ```
- [ ] Nient'altro da cambiare: le spalle restano in posizione (`kp=1`, `kd=1`, velocità 0), come oggi.

#### Nuova funzione `write_transition_wheel_commands(double t, double duration)`
- [ ] Logica:
  - se `transition_wheels_mode_ == "lock"` **oppure** `transition_target_ == TARGET_ACTIVATION`
    **oppure** `duration <= 0` → **esattamente il codice attuale** (velocità 0, pos NaN, eff 0, `kp=0`, `kd=1`);
  - altrimenti (`assist` su `TARGET_STAND` / `TARGET_REST`), per ogni ruota:
    ```
    hfe      = wheel_to_hfe_[ruota]
    q_target = q_stand[hfe] (STAND) oppure q_rest[hfe] (REST)
    q_dot    = (q_target − transition_q_start_[hfe]) · 0.5·π·sin(π·t) / duration
    ω        = −transition_wheels_gain_ · q_dot
    ```
    e comando: `velocity = ω`, `position = NaN`, `_effort = 0`, `kp_scale = 0`, `kd_scale = 1`.
  - In `sim_flag_` impostare solo l'interfaccia `velocity`, come fa già il codice attuale.
- [ ] Il calcolo di `q_target` deve usare la stessa logica del loop delle gambe
  (`joint_targets_.count(jnt) ? ... : 0.0`). Meglio estrarre un piccolo helper `transition_target_q(jnt)`
  da usare in entrambi i punti, così non può divergere.

#### Da NON toccare
- `stand_service_cb()` / `rest_service_cb()` / `activate_service_cb()`: la logica di ingresso nelle transizioni resta uguale.
- La gestione di fine transizione (`if (t >= 1.0)`): al passaggio in ACTIVE le ruote tornano a `write_wheel_commands()`.
- `wheel_ik.cpp`: il moto dei cingoli in salita **non** è una twist della base e non deve passare dalla `WheelIK`.

### 2.3 `src/pi3hat/pi3hat_hw_interface/config/<traquad>.yaml` (non committato, quello sul robot)
- [ ] Aggiungere sotto `omni_controller.ros__parameters`:
  ```yaml
  transition_wheels:
    mode: assist   # "lock" = comportamento attuale (ruote frenate)
    gain: 7.4      # k: ω_cingolo = −k · q̇_spalla
  ```
- [ ] Controllare che `joint_targets.<X>_HFE.q_stand` abbia i segni **LF +, LH −, RF −, RH +**, coerenti con `getup_traq.py`.
- [ ] Controllare `stand_duration` / `rest_duration` (> 0).

### 2.4 `omniquad12.yaml` / `omniquad12_dji.yaml` / `omnicar*.yaml`
- [ ] **Nessuna modifica.** Senza `transition_wheels` il default è `lock`, cioè il comportamento identico a oggi.

### 2.5 `src/omni_controller/README.md`
- [ ] Documentare i parametri `transition_wheels.mode` e `transition_wheels.gain`, la formula, che è attivo
  solo su STAND/REST e mai su ACTIVATION, e il fatto che il default è `lock`.

---

## 3. Lato NUC (questo workspace)

- [ ] **Nessuna modifica necessaria**: △ continua a chiamare `/omni_controller/stand_srv`
  (`omni_mulinex_joystick.cpp`, ~riga 240).
- [ ] `getup_traq.py` / `getup_omni.py` diventano superflui per lo stand: da decidere se rimuoverli.

---

## 4. Prove, in quest'ordine

1. [ ] **Regressione omniquad**: YAML invariata, si fa uno stand registrando un bag.
   Su `/omni_controller/debug/joints_command` le ruote durante la transizione devono avere velocità 0 e `kd_scale` 1, come prima.
   All'avvio il log deve riportare `mode=lock`.
2. [ ] **Traquad sollevato** (cingoli staccati da terra), `mode: assist`, `gain` basso (es. 2.0):
   - premere △ e verificare in `/omni_controller/debug/joints_command` che **ω_cingolo abbia segno opposto a q̇ della spalla** su tutte e 4 le gambe;
   - verificare che ω parta da 0, abbia il massimo a metà e torni a 0;
   - premere ✕ e verificare che i segni si invertano.
3. [ ] **Traquad a terra**, `gain` basso, poi aumentare verso ~7.4 osservando lo strisciamento dei cingoli
   e la corrente delle spalle (`/omni_controller/joints_state`, campo `current`).
4. [ ] Controllare che **L1** (activation) non muova i cingoli.
5. [ ] Controllare la safety: se scatta CRITICAL durante la salita, i cingoli si devono fermare
   (`update_damping()` li blocca già, nessuna modifica prevista).

---

## 5. Verifica dei segni (da `getup_traq.py`)

| Gamba | `q_stand` spalla (da 0, quindi segno di q̇) | Velocità ruota in `getup_traq.py` | Opposta? |
|---|---|---|---|
| LF | +1.2 → q̇ > 0 | −1.79 rad/s | ✓ |
| LH | −1.2 → q̇ < 0 | +1.79 rad/s | ✓ |
| RF | −1.2 → q̇ < 0 | +1.79 rad/s | ✓ |
| RH | +1.2 → q̇ > 0 | −1.79 rad/s | ✓ |

Quindi `ω_cingolo = −k · q̇_spalla` con **un solo k positivo** per tutte le gambe.

---

## 6. Da decidere più avanti (fuori scopo)

- Feedforward di velocità sulle spalle (oggi `velocity = 0` con `kd=1` frena leggermente il moto):
  si potrebbe passare `q̇` come feedforward. Non necessario per ora.
- Usare `assist` anche sull'omniquad 12 DoF: solo dopo una prova e cambiando **solo** la sua YAML.
