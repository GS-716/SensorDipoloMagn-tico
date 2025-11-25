# -*- coding: utf-8 -*-
"""
Created on Tue Nov 25 01:28:39 2025

@author: Nicole
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # solo para activar proyección 3d

# ---------------------------
# Parámetros físicos del imán
# ---------------------------
mu0 = 4 * np.pi * 1e-7  # H/m

# Dimensiones del imán (en metros)
diameter = 30e-3       # 30 mm
radius   = diameter/2  # radio
length   = 6e-3        # 6 mm

# Campo de remanencia típico de un imán de neodimio (puedes ajustarlo)
Br = 1.2  # Tesla aprox.

# Magnetización (suponiendo imán uniformemente magnetizado)
M = Br / mu0  # A/m

# Momento magnético efectivo: m = M * volumen
V  = np.pi * radius**2 * length
m_mag = M * V
m_vec = np.array([0.0, 0.0, m_mag])  # orientado en +z

# ---------------------------
# Campo de un dipolo en 3D
# B(r) = mu0/(4π) [3 (m·r_hat) r_hat - m] / r^3
# ---------------------------
def B_dipole(r, m=m_vec):
    r = np.array(r, dtype=float)
    r_norm = np.linalg.norm(r)
    if r_norm == 0:
        return np.zeros(3)
    r_hat = r / r_norm
    factor = mu0 / (4*np.pi * r_norm**3)
    return factor * (3 * np.dot(m, r_hat) * r_hat - m)

# ---------------------------
# Integración de líneas de campo
# ---------------------------
def trace_field_line(r0, h=0.002, n_steps=500, direction=1, r_max=0.15):
    """
    Integra una línea de campo a partir de r0.
    h: tamaño de paso (m)
    direction: +1 o -1 (dos sentidos de la línea)
    r_max: radio máximo antes de detener la integración
    """
    pts = []
    r = np.array(r0, dtype=float)
    for _ in range(n_steps):
        B = B_dipole(r)
        B_norm = np.linalg.norm(B)
        if B_norm == 0:
            break
        # Paso proporcional a la dirección del campo (normalizado)
        r = r + direction * h * B / B_norm
        if np.linalg.norm(r) > r_max:
            break
        pts.append(r.copy())
    return np.array(pts)

# ---------------------------
# Generar varias líneas de campo
# ---------------------------
# Semillas alrededor de las “caras” del imán
seed_radius = 1.3 * radius   # un poco fuera de la superficie
z_top  = +length/2
z_bot  = -length/2
n_seeds = 12

seeds = []
for theta in np.linspace(0, 2*np.pi, n_seeds, endpoint=False):
    x = seed_radius * np.cos(theta)
    y = seed_radius * np.sin(theta)
    seeds.append([x, y, z_top + 1e-3])  # un poquito por encima
    seeds.append([x, y, z_bot - 1e-3])  # un poquito por debajo

# ---------------------------
# Plot 3D
# ---------------------------
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')

# Dibujar el imán como cilindro
z_cyl = np.linspace(-length/2, length/2, 30)
theta_cyl = np.linspace(0, 2*np.pi, 40)
Theta, Zc = np.meshgrid(theta_cyl, z_cyl)
Xc = radius * np.cos(Theta)
Yc = radius * np.sin(Theta)
ax.plot_surface(Xc, Yc, Zc, alpha=0.3, linewidth=0)

# Dibujar líneas de campo
for s in seeds:
    # hacia afuera del imán
    pts_plus = trace_field_line(s, direction=+1)
    if len(pts_plus) > 0:
        ax.plot3D(pts_plus[:,0], pts_plus[:,1], pts_plus[:,2])
    # hacia adentro / otro sentido
    pts_minus = trace_field_line(s, direction=-1)
    if len(pts_minus) > 0:
        ax.plot3D(pts_minus[:,0], pts_minus[:,1], pts_minus[:,2])

# Ajustes de la figura
ax.set_xlabel('x (m)')
ax.set_ylabel('y (m)')
ax.set_zlabel('z (m)')
ax.set_title('Líneas de campo de un imán cilíndrico (aprox. dipolo)')

# Ejes con la misma escala para que no se deforme
lims = 0.12  # 12 cm alrededor del imán
ax.set_xlim(-lims, lims)
ax.set_ylim(-lims, lims)
ax.set_zlim(-lims, lims)
ax.set_box_aspect([1, 1, 1])

plt.tight_layout()
plt.show()

