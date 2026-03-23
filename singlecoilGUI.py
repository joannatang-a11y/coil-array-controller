def run gui():
    root = tk.Tk()
    root.title("Magnetic Sites Sequencer (32) — SEQ delay + SPLIT delay")

    topbar = ttk.Frame(root)
    topbar.pack(fill="x", padx=10, pady=8)

    conn_text_var = tk.StringVar(value="Connected: NO")
    status_top_var = tk.StringVar(value="Ready (TEST MODE)" if TEST_MODE else "Ready (Not connected)")

    port_var = tk.StringVar(value=DEFAULT_COM_PORT)
    ports_list = list_com_ports()
    if ports_list and DEFAULT_COM_PORT not in ports_list:
        port_var.set(ports_list[0])

    def refresh_ports():
        ports = list_com_ports()
        port_menu["values"] = ports if ports else [DEFAULT_COM_PORT]
        if ports and port_var.get() not in ports:
            port_var.set(ports[0])

    def refresh_conn_text():
        conn_text_var.set("Connected: YES" if connected else "Connected: NO")

    def on_connect():
        ok = connect_device(port_var.get())
        refresh_conn_text()
        if ok:
            status_top_var.set("Connected (TEST)" if TEST_MODE else f"Connected to {port_var.get()}")
        else:
            status_top_var.set("Connect failed")

    def on_disconnect():
        disconnect_device()
        refresh_conn_text()
        status_top_var.set("Disconnected")

    ttk.Label(topbar, text="Port:").pack(side="left")
    port_menu = ttk.Combobox(topbar, textvariable=port_var, width=10, state="readonly")
    port_menu["values"] = ports_list if ports_list else [DEFAULT_COM_PORT]
    port_menu.pack(side="left", padx=6)

    ttk.Button(topbar, text="Refresh Ports", width=14, command=refresh_ports).pack(side="left", padx=4)
    ttk.Button(topbar, text="Connect", width=12, command=on_connect).pack(side="left", padx=4)
    ttk.Button(topbar, text="Disconnect", width=12, command=on_disconnect).pack(side="left", padx=4)
    ttk.Label(topbar, textvariable=conn_text_var).pack(side="left", padx=12)
    ttk.Label(topbar, textvariable=status_top_var).pack(side="left", padx=12)

    nb = ttk.Notebook(root)
    nb.pack(fill="both", expand=True, padx=6, pady=6)

    single_tab = SingleFluidTab(nb, conn_text_var)
    dual_tab = DualFluidTab(nb, conn_text_var)

    nb.add(single_tab.frame, text="Single Travel + Split (separate timing)")
    nb.add(dual_tab.frame, text="Dual Fluid (2 pathways)")

    refresh_conn_text()
    root.mainloop()
