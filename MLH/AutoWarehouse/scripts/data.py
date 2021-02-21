from datetime import datetime as dt
from enum import Enum


class D(object):

    class Team:
        Id = "VB#0332"
        UniqueId = "sBgMeYfW"

    @staticmethod
    def StorageNumber(package):
        return "R{} C{}".format(package[-2], package[-1])

    @staticmethod
    def SKU(package, color):
        today = dt.today()
        month = today.month if len(
            str(today.month)) < 2 else "0{}".format(today.month)
        year = str(today.year)

        _sku = "{}{}{}{}".format(
            color[0].upper(),
            package[-2:],
            month,
            year[-2:]
        )

        return _sku

    # Google Sheets
    class Sheet:
        Inventory = "Inventory"
        Orders = "IncomingOrders"
        Dispatch = "OrdersDispatched"
        Ship = "OrdersShipped"
        Dashboard = "Dashboard"

    class Priority(Enum):
        red = "HP"
        yellow = "MP"
        green = "LP"

    class Item(Enum):
        red = "Medicine"
        yellow = "Packaged Food"
        green = "Clothes"

    class Cost(Enum):
        red = 1200
        yellow = 750
        green = 560

    class OItem:
        HP = "Medicine"
        MP = "Packaged Food"
        LP = "Clothes"

    PColor = {
        'Medicine': 'red',
        'Food': 'yellow',
        'Clothes': 'green'
    }

    def create_inventory_data(self, package, color):
        """
        Creates a list of parameters to be updated in Warehouse Management 
        Spreadsheet in Inventory sheet.
        """

        data = {
            "id": self.Sheet.Inventory,
            "Team Id": self.Team.Id,
            "Unque Id": self.Team.UniqueId,
            "SKU": self.SKU(package, color),
            "Item": self.Item[color].value,
            "Priority": self.Priority[color].value,
            "Storage Number": self.StorageNumber(package),
            "Cost": self.Cost[color].value,
            "Quantity": 1
        }

        return data

    def create_dispatch_data(self):
        """
        Creates a list of parameters to be updated in Warehouse Management 
        Spreadsheet in OrdersDispatched sheet.
        """

        data = {

        }

        return data
