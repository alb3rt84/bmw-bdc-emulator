"""FA and VCM XML used by the ZGW window.

E-Sys saves the vehicle order as faList / standardFA. Namespaces are ignored.
The VCM file is the one this program writes and reads: VIN, I-Stufe and the
same order. fa_bytes() is the version-3 blob answered on UDS 22 3F 06.
"""

from __future__ import annotations

import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

_CODE4 = re.compile(r"^[A-Z0-9]{4}$")
_CODE3 = re.compile(r"^[A-Z0-9]{3}$")
_ISTUFE = re.compile(r"^[A-Z0-9]{4}-\d{2}-\d{2}-\d{3}$")
_VIN_BAD = re.compile(r"[IOQ]")
MAX_FA_BYTES = 480


def _local(tag: str) -> str:
    if tag.startswith("{"):
        return tag.split("}", 1)[1]
    return tag


def _find(el: ET.Element, name: str) -> ET.Element | None:
    for node in el.iter():
        if _local(node.tag) == name:
            return node
    return None


def _texts(parent: ET.Element | None, child: str) -> list[str]:
    if parent is None:
        return []
    out: list[str] = []
    for node in parent:
        if _local(node.tag) != child:
            continue
        text = (node.text or "").strip().upper()
        if text:
            out.append(text)
    return out


def _need4(value: str, label: str) -> str:
    value = value.strip().upper()
    if not _CODE4.match(value):
        raise ValueError(f"{label} ma mieć 4 znaki A–Z 0–9, jest {value!r}.")
    return value


def _vin_ok(vin: str) -> str:
    vin = "".join(vin.split()).upper()
    if len(vin) != 17 or _VIN_BAD.search(vin) or not vin.isalnum():
        raise ValueError("VIN w XML ma mieć 17 znaków, bez liter I, O i Q.")
    return vin


def _istufe_ok(value: str, label: str) -> str:
    value = value.strip().upper()
    if not value:
        return ""
    if not _ISTUFE.match(value):
        raise ValueError(f"{label} ma wyglądać jak F025-18-03-520.")
    return value


@dataclass
class Fa:
    series: str
    type_key: str
    time_criteria: str
    colour: str
    fabric: str
    e_codes: list[str] = field(default_factory=list)
    sa_codes: list[str] = field(default_factory=list)
    ho_codes: list[str] = field(default_factory=list)
    vin: str = ""

    def summary(self) -> str:
        return f"{self.series} {self.type_key} {self.time_criteria}"


@dataclass
class Vcm:
    vin: str = ""
    i_stufe: str = ""
    i_stufe_werk: str = ""
    i_stufe_ho: str = ""
    fa: Fa | None = None


def fa_from_element(root: ET.Element) -> Fa:
    header = _find(root, "header")
    vin = ""
    if header is not None:
        raw = header.attrib.get("vinLong") or header.attrib.get("vin") or ""
        if raw.strip():
            vin = _vin_ok(raw)

    std = root if _local(root.tag) == "standardFA" else _find(root, "standardFA")
    if std is None:
        raise ValueError("W pliku nie ma standardFA.")

    version = (std.attrib.get("faVersion") or "3").strip()
    if version != "3":
        raise ValueError("Emulator obsługuje FA w wersji 3.")

    fa = Fa(
        series=_need4(std.attrib.get("series", ""), "Seria"),
        type_key=_need4(std.attrib.get("typeKey", ""), "Typ"),
        time_criteria=_need4(std.attrib.get("timeCriteria", ""), "Data"),
        colour=_need4(std.attrib.get("colourCode", ""), "Lakier"),
        fabric=_need4(std.attrib.get("fabricCode", ""), "Tapicerka"),
        vin=vin,
    )
    fa.e_codes = _texts(_find(std, "eCodes"), "eCode")
    fa.sa_codes = _texts(_find(std, "saCodes"), "saCode")
    fa.ho_codes = _texts(_find(std, "hoCodes"), "hoCode")
    for code in fa.e_codes:
        if not _CODE4.match(code):
            raise ValueError(f"E-Wort {code!r} ma mieć 4 znaki.")
    for code in fa.sa_codes:
        if not _CODE3.match(code):
            raise ValueError(f"SA {code!r} ma mieć 3 znaki.")
    for code in fa.ho_codes:
        if not _CODE4.match(code):
            raise ValueError(f"HO {code!r} ma mieć 4 znaki.")
    if len(fa.e_codes) > 255 or len(fa.sa_codes) > 255 or len(fa.ho_codes) > 255:
        raise ValueError("Za dużo kodów w FA.")
    raw = fa_bytes(fa)
    if len(raw) > MAX_FA_BYTES:
        raise ValueError("FA jest za duże na emulator (maks. 480 bajtów).")
    return fa


def load_fa(path: str | Path) -> Fa:
    root = ET.parse(path).getroot()
    tag = _local(root.tag)
    if tag not in ("faList", "fa", "standardFA"):
        raise ValueError("To nie jest plik FA (oczekiwany faList).")
    return fa_from_element(root)


def load_vcm(path: str | Path) -> Vcm:
    root = ET.parse(path).getroot()
    if _local(root.tag) != "vcm":
        raise ValueError("To nie jest plik VCM (oczekiwany korzeń vcm).")

    def text(name: str) -> str:
        node = _find(root, name)
        return (node.text or "").strip() if node is not None else ""

    vcm = Vcm(
        i_stufe=_istufe_ok(text("iStufe"), "I-Stufe"),
        i_stufe_werk=_istufe_ok(text("iStufeWerk"), "I-Stufe werk"),
        i_stufe_ho=_istufe_ok(text("iStufeHo"), "I-Stufe HO"),
    )
    if text("vin"):
        vcm.vin = _vin_ok(text("vin"))
    if _find(root, "standardFA") is not None:
        vcm.fa = fa_from_element(root)
        if vcm.fa.vin and not vcm.vin:
            vcm.vin = vcm.fa.vin
    return vcm


def fa_bytes(fa: Fa) -> bytes:
    out = bytearray()
    out.append(0x03)
    out += fa.series.encode("ascii")
    out += fa.type_key.encode("ascii")
    out += fa.time_criteria.encode("ascii")
    out += fa.colour.encode("ascii")
    out += fa.fabric.encode("ascii")
    out.append(len(fa.e_codes))
    for code in fa.e_codes:
        out += code.encode("ascii")
    out.append(len(fa.sa_codes))
    for code in fa.sa_codes:
        out += code.encode("ascii")
    out.append(len(fa.ho_codes))
    for code in fa.ho_codes:
        out += code.encode("ascii")
    return bytes(out)


def _self_check() -> None:
    here = Path(__file__).resolve().parent
    fa = load_fa(here / "data" / "FA.xml")
    raw = fa_bytes(fa)
    prefix = bytes.fromhex("03463031354b52323330343138303636384c4353570141303930")
    if raw[: len(prefix)] != prefix or len(raw) != 70:
        raise SystemExit(f"FA.xml koduje się inaczej niż szablon: {len(raw)} {raw[:26].hex()}")
    if fa.summary() != "F015 KR23 0418":
        raise SystemExit(fa.summary())
    vcm = load_vcm(here / "data" / "VCM.xml")
    if vcm.vin != "WBA00000200000000":
        raise SystemExit(vcm.vin)
    if vcm.i_stufe != "F025-18-03-520":
        raise SystemExit(vcm.i_stufe)
    if vcm.fa is None or fa_bytes(vcm.fa) != raw:
        raise SystemExit("VCM.xml ma inne FA niż FA.xml")
    print(f"ok {len(raw)} {fa.summary()} {vcm.i_stufe}")


if __name__ == "__main__":
    _self_check()
