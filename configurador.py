#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
  Rover Explorer — Configurador
  Aplicación para configurar la simulación del rover.
  Modifica directamente el archivo .wbt con los parámetros.
  
  Uso: python configurador.py
═══════════════════════════════════════════════════════════════
"""

import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import os, re

class RoverConfigurador:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("🤖 Rover Explorer — Configurador")
        self.root.geometry("520x620")
        self.root.configure(bg="#0f172a")
        self.root.resizable(False, False)

        self.wbt_path = tk.StringVar(value="")
        self.n_obs = tk.IntVar(value=5)
        self.n_samples = tk.IntVar(value=3)

        self._auto_find_wbt()
        self._build_ui()

    def _auto_find_wbt(self):
        """Busca automáticamente el .wbt en rutas comunes."""
        candidates = [
            os.path.join(os.path.dirname(__file__), "worlds", "rover_explorer.wbt"),
            os.path.join(os.path.dirname(__file__), "..", "worlds", "rover_explorer.wbt"),
            os.path.join(os.getcwd(), "worlds", "rover_explorer.wbt"),
            os.path.join(os.getcwd(), "rover_project", "worlds", "rover_explorer.wbt"),
        ]
        for c in candidates:
            if os.path.exists(c):
                self.wbt_path.set(os.path.abspath(c))
                break

    def _build_ui(self):
        # ── Header ──
        header = tk.Frame(self.root, bg="#0f172a")
        header.pack(fill="x", padx=20, pady=(20, 5))
        tk.Label(header, text="WEBOTS R2025a", font=("Consolas", 9),
                 fg="#22d3ee", bg="#0f172a").pack()
        tk.Label(header, text="🤖 Rover Explorer", font=("Segoe UI", 20, "bold"),
                 fg="#f1f5f9", bg="#0f172a").pack()
        tk.Label(header, text="Configurador de Simulación", font=("Consolas", 10),
                 fg="#64748b", bg="#0f172a").pack()

        # ── Archivo .wbt ──
        file_frame = tk.LabelFrame(self.root, text=" Archivo de mundo ", font=("Consolas", 9),
                                    fg="#94a3b8", bg="#1e293b", bd=1)
        file_frame.pack(fill="x", padx=20, pady=(15, 5))

        path_frame = tk.Frame(file_frame, bg="#1e293b")
        path_frame.pack(fill="x", padx=10, pady=8)
        tk.Entry(path_frame, textvariable=self.wbt_path, font=("Consolas", 8),
                 bg="#0f172a", fg="#22d3ee", insertbackground="#22d3ee",
                 relief="flat", bd=0).pack(side="left", fill="x", expand=True, ipady=4)
        tk.Button(path_frame, text="📂", command=self._browse_wbt,
                  font=("Segoe UI", 10), bg="#334155", fg="#e2e8f0",
                  relief="flat", cursor="hand2").pack(side="right", padx=(5, 0))

        # ── Presets ──
        preset_frame = tk.LabelFrame(self.root, text=" Presets ", font=("Consolas", 9),
                                      fg="#94a3b8", bg="#1e293b", bd=1)
        preset_frame.pack(fill="x", padx=20, pady=5)

        btn_frame = tk.Frame(preset_frame, bg="#1e293b")
        btn_frame.pack(fill="x", padx=10, pady=8)

        presets = [
            ("Fácil\n3 obs · 2 obj", 3, 2),
            ("Normal\n5 obs · 3 obj", 5, 3),
            ("Difícil\n8 obs · 5 obj", 8, 5),
            ("Extremo\n12 obs · 7 obj", 12, 7),
        ]
        for text, obs, samp in presets:
            tk.Button(btn_frame, text=text, font=("Consolas", 8),
                      bg="#0f172a", fg="#e2e8f0", activebackground="#22d3ee",
                      relief="flat", cursor="hand2", width=12, height=2,
                      command=lambda o=obs, s=samp: self._set_preset(o, s)
                      ).pack(side="left", padx=3, expand=True, fill="x")

        # ── Sliders ──
        slider_frame = tk.LabelFrame(self.root, text=" Parámetros ", font=("Consolas", 9),
                                      fg="#94a3b8", bg="#1e293b", bd=1)
        slider_frame.pack(fill="x", padx=20, pady=5)

        # Obstáculos
        obs_frame = tk.Frame(slider_frame, bg="#1e293b")
        obs_frame.pack(fill="x", padx=10, pady=(10, 0))
        tk.Label(obs_frame, text="Obstáculos:", font=("Consolas", 10),
                 fg="#f97316", bg="#1e293b").pack(side="left")
        self.obs_label = tk.Label(obs_frame, text="5", font=("Consolas", 14, "bold"),
                                   fg="#f97316", bg="#1e293b")
        self.obs_label.pack(side="right")

        obs_slider = tk.Scale(slider_frame, from_=0, to=15, orient="horizontal",
                               variable=self.n_obs, command=self._update_obs,
                               bg="#1e293b", fg="#e2e8f0", troughcolor="#0f172a",
                               highlightthickness=0, sliderrelief="flat",
                               font=("Consolas", 8), length=400)
        obs_slider.pack(padx=10, pady=(0, 5))

        # Muestras
        samp_frame = tk.Frame(slider_frame, bg="#1e293b")
        samp_frame.pack(fill="x", padx=10, pady=(5, 0))
        tk.Label(samp_frame, text="Muestras:", font=("Consolas", 10),
                 fg="#22d3ee", bg="#1e293b").pack(side="left")
        self.samp_label = tk.Label(samp_frame, text="3", font=("Consolas", 14, "bold"),
                                    fg="#22d3ee", bg="#1e293b")
        self.samp_label.pack(side="right")

        samp_slider = tk.Scale(slider_frame, from_=1, to=10, orient="horizontal",
                                variable=self.n_samples, command=self._update_samp,
                                bg="#1e293b", fg="#e2e8f0", troughcolor="#0f172a",
                                highlightthickness=0, sliderrelief="flat",
                                font=("Consolas", 8), length=400)
        samp_slider.pack(padx=10, pady=(0, 10))

        # ── Preview ──
        self.preview_label = tk.Label(self.root, font=("Consolas", 10),
                                       fg="#22d3ee", bg="#0f172a")
        self._update_preview()
        self.preview_label.pack(pady=5)

        # ── Botones de acción ──
        action_frame = tk.Frame(self.root, bg="#0f172a")
        action_frame.pack(fill="x", padx=20, pady=10)

        tk.Button(action_frame, text="✅  Aplicar y Guardar .wbt",
                  font=("Segoe UI", 12, "bold"),
                  bg="#0891b2", fg="white", activebackground="#22d3ee",
                  relief="flat", cursor="hand2", height=2,
                  command=self._apply).pack(fill="x", pady=(0, 5))

        tk.Button(action_frame, text="📋  Copiar línea al portapapeles",
                  font=("Segoe UI", 10),
                  bg="#334155", fg="#e2e8f0", activebackground="#475569",
                  relief="flat", cursor="hand2",
                  command=self._copy).pack(fill="x")

        # ── Status ──
        self.status = tk.Label(self.root, text="", font=("Consolas", 9),
                                fg="#22c55e", bg="#0f172a")
        self.status.pack(pady=5)

    def _set_preset(self, obs, samp):
        self.n_obs.set(obs)
        self.n_samples.set(samp)
        self._update_obs(obs)
        self._update_samp(samp)

    def _update_obs(self, val):
        self.obs_label.config(text=str(self.n_obs.get()))
        self._update_preview()

    def _update_samp(self, val):
        self.samp_label.config(text=str(self.n_samples.get()))
        self._update_preview()

    def _update_preview(self):
        line = f'controllerArgs ["{self.n_obs.get()}", "{self.n_samples.get()}"]'
        self.preview_label.config(text=line)

    def _browse_wbt(self):
        path = filedialog.askopenfilename(
            title="Seleccionar archivo .wbt",
            filetypes=[("Webots World", "*.wbt"), ("Todos", "*.*")]
        )
        if path:
            self.wbt_path.set(path)

    def _get_line(self):
        return f'  controllerArgs ["{self.n_obs.get()}", "{self.n_samples.get()}"]'

    def _apply(self):
        path = self.wbt_path.get()
        if not path or not os.path.exists(path):
            messagebox.showerror("Error", "Selecciona un archivo .wbt válido")
            return

        try:
            with open(path, 'r', encoding='utf-8') as f:
                content = f.read()

            # Buscar y reemplazar la línea controllerArgs
            pattern = r'controllerArgs\s*\[.*?\]'
            new_args = f'controllerArgs ["{self.n_obs.get()}", "{self.n_samples.get()}"]'

            if re.search(pattern, content):
                content = re.sub(pattern, new_args, content)
            else:
                messagebox.showerror("Error",
                    "No se encontró 'controllerArgs' en el archivo .wbt.\n"
                    "Asegúrate de usar el archivo rover_explorer.wbt correcto.")
                return

            with open(path, 'w', encoding='utf-8') as f:
                f.write(content)

            self.status.config(text=f"✅ Guardado: {self.n_obs.get()} obs, {self.n_samples.get()} muestras",
                               fg="#22c55e")
            messagebox.showinfo("Éxito",
                f"Archivo actualizado correctamente:\n"
                f"  Obstáculos: {self.n_obs.get()}\n"
                f"  Muestras: {self.n_samples.get()}\n\n"
                f"Reinicia la simulación en Webots para aplicar.")
        except Exception as e:
            messagebox.showerror("Error", f"No se pudo escribir el archivo:\n{e}")

    def _copy(self):
        line = self._get_line()
        self.root.clipboard_clear()
        self.root.clipboard_append(line)
        self.status.config(text="📋 Copiado al portapapeles", fg="#22d3ee")

    def run(self):
        self.root.mainloop()


if __name__ == "__main__":
    RoverConfigurador().run()
