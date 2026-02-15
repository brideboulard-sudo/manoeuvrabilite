# MMG 3-DOF – Ship Maneuvering Lab

Ce projet implémente un modèle de manœuvrabilité navire 3-DOF (MMG standard method) pour comparer des résultats numériques avec des données expérimentales (KVLCC2), évaluer des critères IMO, inclure les efforts de vent (Isherwood), et estimer une surface minimale de safran.

---

## 1) Structure du projet

- `main.py` : script principal (configuration des essais, exécution, tracés, comparaison exp/num, critères IMO, sizing safran).
- `mmg_3dof.py` : cœur du modèle dynamique (solveur RK4, classes `Ship`, `Hull`, `Propeller`, `Rudder`, `Wind`).
- `ship_params.py` : paramètres navire KVLCC2 (géométrie, dérivées hydrodynamiques, masses ajoutées, hélice, safran, vent).
- `data_exp/` : fichiers expérimentaux (`traj_*`, `U_*`, `r_*`, `beta_*`, `delta_ZZ10`, `psi_ZZ10`).
- `figures/` : figures générées à chaque run.

---

## 2) Modèle physique implémenté

Le modèle considère les 3 degrés de liberté horizontaux :
- surge `u`, sway `v_m`, yaw rate `r`
- position cap dans le repère fixe : `x_s`, `y_s`, `psi`

Les équations intégrées sont de forme :

- `eta_dot = R(psi) * mu`
- `mu_dot = M^-1 * (T - C(mu)*mu)`

avec :
- `eta = [x_s, y_s, psi]`
- `mu = [u, v_m, r]`
- `T = T_hull + T_propeller + T_rudder + T_wind`

### 2.1 Matrice de masse et Coriolis

Dans `Ship.__init__` et `Ship.diff_eq` :
- `M` est construite avec masse, masses ajoutées (`m_x`, `m_y`) et inertie (`I_zG`, `J_z`).
- `C(mu)` dépend de `r` (termes non linéaires de couplage inertiel).

### 2.2 Efforts coque (`Hull.compute_T`)

- Vitesse totale : `U = sqrt(u^2 + v_m^2)`
- Variables réduites : `v' = v_m/U`, `r' = r*Lpp/U`
- Polynômes MMG pour `X'_H`, `Y'_H`, `N'_H` via les dérivées de `ship_params.py`
- Remise à l’échelle dimensionnelle :
  - `X_H = 0.5*rho*Lpp*d*U^2*X'_H`
  - `Y_H = 0.5*rho*Lpp*d*U^2*Y'_H`
  - `N_H = 0.5*rho*Lpp^2*d*U^2*N'_H`

### 2.3 Efforts hélice (`Propeller.compute_T`)

- Calcul de `J_p` (advance ratio) avec le wake en manœuvre.
- `K_T = k0 + k1*J_p + k2*J_p^2`.
- Poussée `T_p = rho * n_p^2 * D_p^4 * K_T`.
- Contribution MMG : `X_P = (1 - t_p) * T_p`, `Y_P = 0`, `N_P = 0`.

### 2.4 Efforts safran (`Rudder.compute_T`)

- Calcul de l’écoulement effectif au safran (`u_r`, `v_r`, `U_r`, `alpha_r`).
- Force normale `F_n` puis efforts/moment : `X_R`, `Y_R`, `N_R`.
- Saturation de vitesse angulaire de barre via `rr_max` dans `set_delta`.

### 2.5 Efforts de vent (`Wind.compute_T`)

Le modèle Isherwood est actif dans `Ship` :
- vitesse/direction vent dans `ship_params['ship_wind']`
- vent apparent calculé dans le repère navire
- coefficients aérodynamiques `C_X`, `C_Y`, `C_N` obtenus par interpolation
- efforts appliqués :
  - `X_w ~ 0.5 * rho_air * A_T * C_X * U_app^2`
  - `Y_w ~ 0.5 * rho_air * A_L * C_Y * U_app^2`
  - `N_w ~ 0.5 * rho_air * A_L * Lpp * C_N * U_app^2`

> Interprétation : la “résistance due au vent” en longitudinal correspond principalement à `X_w`.

---

## 3) Solveur numérique

`DynSolver.solve_dyn()` :
- initialise et stocke explicitement l’état initial à `t=0`
- boucle temporelle jusqu’à `t_max`
- met à jour la commande (`controler`) puis intègre un pas RK4 (`solve_rk4`)
- retourne `t`, `eta`, `mu`, `rudder_delta`

Le pas de temps est défini dans `main.py` via `dyn_params` (`d_t=1 s` par défaut).

---

## 4) Manœuvres simulées

Dans `main.py` :
- `turning_circle` (ex. 35°)
- `zig_zag` (ex. 10/10)

Le contrôleur de barre (`Ship.controler`) :
- turning circle : angle constant
- zig-zag : inversion du signe de consigne après franchissement du cap cible

---

## 5) Comparaison Expérimental vs Numérique

Le script charge les fichiers `data_exp/*.dat` et trace sur les mêmes graphes :
- trajectoire (`x/Lpp`, `y/Lpp`)
- vitesse réduite `U/u0`
- lacet réduit `rLpp/u0`
- dérive `beta`
- pour ZZ10 : `delta` et `psi`

### Sauvegarde des figures

Au démarrage :
- le dossier `figures/` est nettoyé (écrasement des anciennes images)
- toutes les figures du run sont sauvegardées en PNG numérotés

---

## 6) Critères IMO implémentés

### 6.1 Turning circle +35°

- **Advance** : distance longitudinale au cap ~90°
- **Tactical diameter** : distance latérale au cap ~180°
- limites utilisées : advance <= 4.5 L, tactical <= 5.0 L

### 6.2 Zig-zag 10/10

Calcul des overshoots selon la définition utilisée dans le script :
- `OS1 = psi_max(1er virage) - 10°`
- `OS2 = -10° - psi_min(2e virage)`

Le script affiche num/exp et les limites IMO dépendant de `L/V`.

---

## 7) Stabilité de cap (yaw stability)

`main.py` calcule l’indice :

`S = Y'_v (N'_r - m' x'_G) - N'_v (Y'_r - m')`

- `S > 0` : stable directionnellement
- `S < 0` : instable

Cela sert à la partie 3.2.2 du sujet.

---

## 8) Dimensionnement surface safran (partie 3.3)

Fonction `find_min_rudder_area(...)` :
- applique un facteur de vent (`0, 1, 3, 5` fois `U0`)
- cherche la surface minimale `A_r` qui passe les critères IMO TC35 + ZZ10
- stratégie : bornage + dichotomie

Sorties imprimées : surface minimale et métriques associées.

---

## 9) Paramètres importants à modifier

### Dans `ship_params.py`

- `mu_0` : vitesse initiale
- `hull_der` : dérivées hydro (inclut stabilité directionnelle)
- `rudder['A_r']` : surface de safran
- `ship_wind['vel_wind']` : vitesse vent
- `ship_wind['psi_wind']` : direction vent (degrés)

### Dans `main.py`

- type de manœuvre (`zig_zag` / `turning_circle`)
- angle de barre de l’essai
- paramètres temporels (`t_max`, `d_t`)

---

## 10) Exécution

Depuis le dossier du projet :

```bash
python main.py
```

En environnement sans affichage (pour calcul batch) :

```bash
MPLBACKEND=Agg python main.py
```

(Sous Windows PowerShell : `$env:MPLBACKEND='Agg'; python main.py`)

---

## 11) Limites et points de vigilance

- Les résultats TC35 expérimentaux peuvent être sensibles à la méthode utilisée pour relier trajectoire et cap (échantillonnages différents selon fichiers).
- Les métriques IMO doivent être calculées avec une définition cohérente (surtout OS1/OS2).
- Les paramètres aérodynamiques et surfaces exposées au vent influencent fortement les efforts `Y_w` et `N_w`.

---

## 12) Résumé rapide

Le code résout un modèle MMG 3-DOF non linéaire, combine coque/hélice/safran/vent, compare aux données KVLCC2, vérifie les critères IMO et explore le dimensionnement du safran avec et sans vent.
