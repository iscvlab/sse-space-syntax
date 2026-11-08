# -*- coding: utf-8 -*-
"""
SSE (Space Syntax Engine) QGIS Plugin Lifecycle Manager
Adds processing provider and interface menu hooks (Cleaner + Scatter Plot R² Explorer).
"""

import os
from qgis.core import QgsApplication
from qgis.PyQt.QtGui import QIcon
from qgis.PyQt.QtWidgets import QAction
from .provider import SSEProvider
from .dialog_cleaner import SSECleanerDialog
from .dialog_scatter import SSEScatterDialog


class SSEPlugin:
    def __init__(self, iface):
        self.iface = iface
        self.provider = None
        self.action_cleaner = None
        self.action_scatter = None
        self.dlg_cleaner = None
        self.dlg_scatter = None

    def initProcessing(self):
        self.provider = SSEProvider()
        QgsApplication.processingRegistry().addProvider(self.provider)

    def initGui(self):
        self.initProcessing()
        if self.iface:
            icon_path = os.path.join(os.path.dirname(__file__), "icon.png")
            icon = QIcon(icon_path) if os.path.exists(icon_path) else QIcon()
            self.action_cleaner = QAction(
                icon,
                "Enhanced 5-Step Network Cleaner (Dual-Mode)...",
                self.iface.mainWindow()
            )
            self.action_cleaner.triggered.connect(self.open_cleaner_dialog)
            self.iface.addPluginToVectorMenu(
                "&SSE — Space Syntax Engine", self.action_cleaner
            )
            self.iface.addToolBarIcon(self.action_cleaner)

            self.action_scatter = QAction(
                icon,
                "Space Syntax Scatter Plot (Intelligibility / Synergy R²)...",
                self.iface.mainWindow()
            )
            self.action_scatter.triggered.connect(self.open_scatter_dialog)
            self.iface.addPluginToVectorMenu(
                "&SSE — Space Syntax Engine", self.action_scatter
            )

    def open_cleaner_dialog(self):
        if self.dlg_cleaner is None:
            self.dlg_cleaner = SSECleanerDialog(self.iface)
        self.dlg_cleaner.populate_layers()
        self.dlg_cleaner.show()
        self.dlg_cleaner.raise_()
        self.dlg_cleaner.activateWindow()

    def open_scatter_dialog(self, preferred_layer=None, preferred_x="", preferred_y=""):
        if self.dlg_scatter is None:
            self.dlg_scatter = SSEScatterDialog(self.iface)
        self.dlg_scatter.populate_layers(
            preferred_layer=preferred_layer,
            preferred_x=preferred_x,
            preferred_y=preferred_y
        )
        self.dlg_scatter.show()
        self.dlg_scatter.raise_()
        self.dlg_scatter.activateWindow()

    def unload(self):
        if self.action_cleaner and self.iface:
            self.iface.removePluginVectorMenu(
                "&SSE — Space Syntax Engine", self.action_cleaner
            )
            self.iface.removeToolBarIcon(self.action_cleaner)
            self.action_cleaner = None
        if self.action_scatter and self.iface:
            self.iface.removePluginVectorMenu(
                "&SSE — Space Syntax Engine", self.action_scatter
            )
            self.action_scatter = None
        if self.provider:
            QgsApplication.processingRegistry().removeProvider(self.provider)
            self.provider = None


OpenDepthmapPlugin = SSEPlugin
