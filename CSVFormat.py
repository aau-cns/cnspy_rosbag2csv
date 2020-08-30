from enum import Enum

from tum_eval.TUMCSVheader import TUMCSVheader


class CSVFormat(Enum):
    TUM = 'TUM'
    TUM_short = 'TUM_short'
    PoseCov = 'PoseCov'
    none = 'none'

    def __str__(self):
        return self.value

    @staticmethod
    def get_header(format):
        if str(format) == 'TUM':
            return TUMCSVheader.default()
        elif str(format) == 'TUM_short':
            return TUMCSVheader.pos_stamped()
        elif str(format) == 'PoseCov':
            return ['# t', 'pxx', 'pxy', 'pxz', 'pyy', 'pyz', 'pzz', 'qrr', 'qrp', 'qry', 'qpp', 'qpy', 'qyy']
        else:
            return "# no header "
