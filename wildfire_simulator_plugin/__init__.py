from .plugin import WFSPlugin

def classFactory(iface):
    return WFSPlugin(iface)
