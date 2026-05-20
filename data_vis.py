import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import pandas as pd
import sys
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk

plt.style.use('dark_background')

# Requires: pip install pandas matplotlib mplcursors
import mplcursors

import os

class PlotConfigFrame(ttk.LabelFrame):
    def __init__(self, parent, columns, remove_callback, *args, **kwargs):
        super().__init__(parent, *args, **kwargs)
        self.remove_callback = remove_callback
        
        self.listbox = tk.Listbox(self, selectmode=tk.MULTIPLE, exportselection=False, height=6,
                                  bg='#3c3f41', fg='#ffffff', selectbackground='#5c5c5c', highlightthickness=0)
        self.listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        for col in columns:
            self.listbox.insert(tk.END, col)
            
        scrollbar = ttk.Scrollbar(self, orient=tk.VERTICAL, command=self.listbox.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.listbox.config(yscrollcommand=scrollbar.set)
        
        btn_frame = ttk.Frame(self)
        btn_frame.pack(side=tk.BOTTOM, fill=tk.X, padx=5, pady=5)
        remove_btn = ttk.Button(btn_frame, text="✖ Remove Plot", command=self.destroy_self)
        remove_btn.pack(side=tk.RIGHT)
        
    def get_selected_columns(self):
        return [self.listbox.get(i) for i in self.listbox.curselection()]
        
    def destroy_self(self):
        self.remove_callback(self)
        self.destroy()

class DataVisApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Interactive Rocket Data Visualizer")
        self.root.geometry("1400x800")
        
        # Ensure script exits properly when window is closed
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # Apply Dark Theme styling
        self.root.configure(bg="#2b2b2b")
        style = ttk.Style(self.root)
        style.theme_use('clam')
        style.configure('.', background='#2b2b2b', foreground='#ffffff', fieldbackground='#3c3f41')
        style.configure('TButton', background='#3c3f41', foreground='#ffffff', borderwidth=1)
        style.map('TButton', background=[('active', '#545454')])
        style.configure('TEntry', fieldbackground='#3c3f41', foreground='#ffffff', bordercolor='#2b2b2b')
        style.configure('TLabel', background='#2b2b2b', foreground='#ffffff')
        style.configure('TLabelframe', background='#2b2b2b', foreground='#ffffff', bordercolor='#5c5c5c')
        style.configure('TLabelframe.Label', background='#2b2b2b', foreground='#ffffff')
        style.configure('Vertical.TScrollbar', background='#3c3f41', troughcolor='#2b2b2b', bordercolor='#2b2b2b', arrowcolor='#ffffff')
        
        self.df = None
        self.time_col = "timestamp_ms"
        self.plot_frames = []
        self.cursor = None
        
        self.setup_ui()
        
    def setup_ui(self):
        # Main layout
        self.left_panel = ttk.Frame(self.root, width=320)
        self.left_panel.pack(side=tk.LEFT, fill=tk.Y, padx=15, pady=15)
        
        self.right_panel = ttk.Frame(self.root)
        self.right_panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, pady=15, padx=(0, 15))
        
        # --- Left Panel Controls ---
        # File loading
        load_btn = ttk.Button(self.left_panel, text="📁 1. Load CSV File", command=self.load_csv)
        load_btn.pack(fill=tk.X, pady=(0, 5))
        
        # File status label
        self.file_status_label = ttk.Label(self.left_panel, text="No file loaded", foreground="#aaaaaa", font=("", 9, "italic"))
        self.file_status_label.pack(fill=tk.X, pady=(0, 15))
        
        # Time filtering
        time_frame = ttk.LabelFrame(self.left_panel, text="Time Filter (ms)")
        time_frame.pack(fill=tk.X, pady=(0, 15))
        
        ttk.Label(time_frame, text="Start:").grid(row=0, column=0, padx=5, pady=2, sticky=tk.W)
        self.start_time_var = tk.StringVar()
        ttk.Entry(time_frame, textvariable=self.start_time_var, width=10).grid(row=0, column=1, padx=5, pady=2)
        
        ttk.Label(time_frame, text="End:").grid(row=1, column=0, padx=5, pady=2, sticky=tk.W)
        self.end_time_var = tk.StringVar()
        ttk.Entry(time_frame, textvariable=self.end_time_var, width=10).grid(row=1, column=1, padx=5, pady=2)
        
        # Plot configurations container (scrollable)
        self.plots_container_wrapper = ttk.LabelFrame(self.left_panel, text="2. Subplots Config")
        self.plots_container_wrapper.pack(fill=tk.BOTH, expand=True, pady=(0, 15))
        
        canvas = tk.Canvas(self.plots_container_wrapper, bg='#2b2b2b', highlightthickness=0)
        scrollbar = ttk.Scrollbar(self.plots_container_wrapper, orient="vertical", command=canvas.yview)
        self.plots_container = ttk.Frame(canvas)
        
        self.plots_container.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=self.plots_container, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        
        # Add Plot button
        self.add_plot_btn = ttk.Button(self.left_panel, text="➕ Add Subplot", command=self.add_plot, state=tk.DISABLED)
        self.add_plot_btn.pack(fill=tk.X, pady=(0, 10))
        
        # Draw Plot button
        self.draw_btn = ttk.Button(self.left_panel, text="📊 3. Draw / Update Plots", command=self.draw_plots, state=tk.DISABLED)
        self.draw_btn.pack(fill=tk.X, pady=(0, 5))
        
        # Save Filtered Data button
        self.save_btn = ttk.Button(self.left_panel, text="💾 4. Save Filtered Data", command=self.save_filtered_data, state=tk.DISABLED)
        self.save_btn.pack(fill=tk.X, pady=(0, 5))
        
        # --- Right Panel (Matplotlib Canvas) ---
        self.fig, self.ax = plt.subplots()
        self.fig.patch.set_facecolor('#2b2b2b') # Match the GUI background
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.right_panel)
        
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        
        self.toolbar_frame = ttk.Frame(self.right_panel)
        self.toolbar_frame.pack(side=tk.BOTTOM, fill=tk.X, pady=(10, 0))
        self.toolbar = NavigationToolbar2Tk(self.canvas, self.toolbar_frame)
        
    def load_csv(self):
        filepath = filedialog.askopenfilename(filetypes=[("CSV Files", "*.csv"), ("All Files", "*.*")])
        if not filepath:
            return
            
        try:
            # Assumes comma separated, adjusts if spaces exist
            self.df = pd.read_csv(filepath, skipinitialspace=True)
            
            # Identify time column
            if "timestamp_ms" in self.df.columns:
                self.time_col = "timestamp_ms"
            else:
                self.time_col = self.df.columns[0] # Fallback to first column
                
            # Set default time bounds
            min_t = self.df[self.time_col].min()
            max_t = self.df[self.time_col].max()
            self.start_time_var.set(str(min_t))
            self.end_time_var.set(str(max_t))
            
            # Clear existing plots and add one default
            for p in list(self.plot_frames):
                p.destroy_self()
                
            self.add_plot()
            filename = os.path.basename(filepath)
            self.root.title(f"Interactive Rocket Data Visualizer - {filename}")
            self.file_status_label.config(text=f"Loaded: {filename}", foreground="#55ff55", font=("", 9, "normal"))
            
            # Enable the UI buttons now that data is loaded
            self.add_plot_btn.config(state=tk.NORMAL)
            self.draw_btn.config(state=tk.NORMAL)
            self.save_btn.config(state=tk.NORMAL)
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to load CSV:\n{str(e)}")

    def add_plot(self):
        if self.df is None:
            messagebox.showwarning("Warning", "Please load a CSV file first.")
            return
            
        selectable_cols = [c for c in self.df.columns if c != self.time_col]
        frame = PlotConfigFrame(self.plots_container, selectable_cols, self.remove_plot, text=f"Subplot {len(self.plot_frames)+1}")
        frame.pack(fill=tk.X, pady=5, padx=5)
        self.plot_frames.append(frame)
        
    def remove_plot(self, frame):
        if frame in self.plot_frames:
            self.plot_frames.remove(frame)
            
        # Rename frames
        for i, pf in enumerate(self.plot_frames):
            pf.config(text=f"Subplot {i+1}")

    def draw_plots(self):
        if self.df is None:
            return
            
        try:
            t_start = float(self.start_time_var.get())
            t_end = float(self.end_time_var.get())
        except ValueError:
            messagebox.showerror("Error", "Invalid Time Filter values. Must be numbers.")
            return
            
        # Filter dataframe by time
        mask = (self.df[self.time_col] >= t_start) & (self.df[self.time_col] <= t_end)
        filtered_df = self.df[mask]
        
        if filtered_df.empty:
            messagebox.showwarning("Warning", "No data in the selected time range.")
            return

        # Clear existing figure
        self.fig.clear()
        if self.cursor:
            self.cursor.remove()
            self.cursor = None
        
        plot_configs = [pf.get_selected_columns() for pf in self.plot_frames if pf.get_selected_columns()]
        num_subplots = len(plot_configs)
        
        if num_subplots == 0:
            self.fig.patch.set_facecolor('#2b2b2b')
            self.canvas.draw()
            return
            
        axes = self.fig.subplots(num_subplots, 1, sharex=True)
        self.fig.patch.set_facecolor('#2b2b2b')
        if num_subplots == 1:
            axes = [axes] # ensure it's iterable
            
        time_data = filtered_df[self.time_col]
        lines = []

        for ax, cols in zip(axes, plot_configs):
            for col in cols:
                line, = ax.plot(time_data, filtered_df[col], label=col)
                lines.append(line)
            ax.set_ylabel("Values")
            ax.legend(loc="upper right")
            ax.grid(True)
            ax.ticklabel_format(useOffset=False, style='plain', axis='y')
            
        axes[-1].set_xlabel(f"Time ({self.time_col})")
        axes[-1].ticklabel_format(useOffset=False, style='plain', axis='x')
        self.fig.tight_layout()
        
        # Add hover tooltips using mplcursors
        self.cursor = mplcursors.cursor(lines, hover=True)
        @self.cursor.connect("add")
        def on_add(sel):
            sel.annotation.set_text(f"{sel.artist.get_label()}\nTime: {sel.target[0]:g}\nValue: {sel.target[1]:g}")
            sel.annotation.get_bbox_patch().set(fc="#3c3f41", alpha=0.9, ec="#ffffff")

        self.canvas.draw()

    def save_filtered_data(self):
        if self.df is None:
            messagebox.showwarning("Warning", "Please load a CSV file first.")
            return
            
        try:
            t_start = float(self.start_time_var.get())
            t_end = float(self.end_time_var.get())
        except ValueError:
            messagebox.showerror("Error", "Invalid Time Filter values. Must be numbers.")
            return
            
        # Filter dataframe by time
        mask = (self.df[self.time_col] >= t_start) & (self.df[self.time_col] <= t_end)
        filtered_df = self.df[mask]
        
        if filtered_df.empty:
            messagebox.showwarning("Warning", "No data in the selected time range to save.")
            return
            
        save_path = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV Files", "*.csv"), ("All Files", "*.*")],
            title="Save Filtered Data As"
        )
        if save_path:
            try:
                filtered_df.to_csv(save_path, index=False)
                messagebox.showinfo("Success", f"Successfully saved {len(filtered_df)} rows to:\n{save_path}")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to save file:\n{str(e)}")

    def on_closing(self):
        """Handle window close event to cleanly exit the application."""
        plt.close('all')
        self.root.quit()
        self.root.destroy()
        sys.exit(0)

if __name__ == "__main__":
    root = tk.Tk()
    app = DataVisApp(root)
    root.mainloop()
