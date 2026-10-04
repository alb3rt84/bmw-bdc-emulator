"""FA and SVT XML used by the ZGW window.

E-Sys saves the vehicle order as faList / standardFA. That order is what the
BDC keeps as VCM. SVT is the list of controllers fitted now. Namespaces are
ignored. fa_bytes() is the version-3 blob answered on UDS 22 3F 06.
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


PROCESS_CLASS = {
    "HWEL": 0x01, "HWAP": 0x02, "HWFR": 0x03, "GWTB": 0x04,
    "CAFD": 0x05, "BTLD": 0x06, "FLSL": 0x07, "SWFL": 0x08,
    "SWFF": 0x09, "SWPF": 0x0A, "ONPS": 0x0B, "IBAD": 0x0C,
    "SWFK": 0x0D, "FAFP": 0x0F, "ENTD": 0xA0, "NAVD": 0xA1,
    "FCFN": 0xA2,
}
_PART_LINE = re.compile(
    r"^([A-Z0-9]{4})\s+([0-9A-F]{8})\s+(\d{1,3})[.\s](\d{1,3})[.\s](\d{1,3})$"
)


@dataclass
class Sgbm:
    klass: str
    ident: str
    main: int
    sub: int
    patch: int

    def line(self) -> str:
        return f"{self.klass} {self.ident} {self.main:03d}.{self.sub:03d}.{self.patch:03d}"


@dataclass
class SvtEcu:
    addr: int
    name: str
    parts: list[Sgbm] = field(default_factory=list)
    svk_version: int = 1
    prog_dep: int = 1

    def wire(self) -> bytes:
        out = bytearray()
        for part in self.parts:
            klass = PROCESS_CLASS.get(part.klass)
            if klass is None:
                raise ValueError(f"Nieznana klasa części {part.klass}.")
            out.append(klass)
            out += int(part.ident, 16).to_bytes(4, "big")
            out.append(part.main)
            out.append(part.sub)
            out.append(part.patch)
        return bytes(out)


def parse_part_line(line: str) -> Sgbm:
    text = " ".join(line.replace(",", " ").split()).upper()
    match = _PART_LINE.match(text)
    if not match:
        raise ValueError(
            f"Linia części ma wyglądać jak „HWEL 00004159 013.000.002”, jest {line!r}."
        )
    klass, ident, main, sub, patch = match.groups()
    if klass not in PROCESS_CLASS:
        raise ValueError(f"Nieznana klasa części {klass}.")
    versions = [int(main, 10), int(sub, 10), int(patch, 10)]
    if any(v > 255 for v in versions):
        raise ValueError(f"Wersja części {line!r} nie mieści się w bajcie.")
    return Sgbm(klass, ident, versions[0], versions[1], versions[2])


def parse_part_text(text: str) -> list[Sgbm]:
    parts: list[Sgbm] = []
    for raw in text.splitlines():
        if raw.strip():
            parts.append(parse_part_line(raw))
    if not parts:
        raise ValueError("Sterownik musi mieć choć jedną część.")
    if len(parts) > 80:
        raise ValueError("Za dużo części w jednym sterowniku (maks. 80).")
    return parts


def _part_from_node(node: ET.Element) -> Sgbm:
    fields = { _local(child.tag): (child.text or "").strip() for child in node }
    klass = fields.get("processClass", "").upper()
    ident = fields.get("id", "").upper()
    if klass not in PROCESS_CLASS or not re.fullmatch(r"[0-9A-F]{8}", ident):
        raise ValueError(f"Zła część SVT: {klass} {ident}.")
    versions = []
    for key in ("mainVersion", "subVersion", "patchVersion"):
        value = int(fields.get(key, ""), 10)
        if not 0 <= value <= 255:
            raise ValueError(f"Wersja {key} poza zakresem.")
        versions.append(value)
    return Sgbm(klass, ident, versions[0], versions[1], versions[2])


def load_svt(path: str | Path) -> list[SvtEcu]:
    root = ET.parse(path).getroot()
    if _local(root.tag) != "svt":
        raise ValueError("To nie jest plik SVT (oczekiwany korzeń svt).")
    ecus: list[SvtEcu] = []
    seen: set[int] = set()
    for node in root.iter():
        if _local(node.tag) != "ecu":
            continue
        name = (node.attrib.get("baseVariant") or "").strip().upper()
        if not re.fullmatch(r"[A-Z0-9_]{1,19}", name):
            raise ValueError(f"Nazwa sterownika {name!r} ma mieć 1–19 znaków A–Z 0–9 _.")
        addr = None
        for child in node.iter():
            if _local(child.tag) == "diagnosticAddress":
                addr = int(child.attrib.get("physicalOffsetAsHex") or "", 16)
                break
        if addr is None or not 1 <= addr <= 0xFE or addr == 0xDF:
            raise ValueError(f"{name}: adres diagnostyczny jest poza zakresem.")
        if addr in seen:
            raise ValueError(f"Adres 0x{addr:02X} powtarza się w SVT.")
        seen.add(addr)
        svk = None
        for child in node.iter():
            if _local(child.tag) == "standardSVK":
                svk = child
                break
        if svk is None:
            raise ValueError(f"{name}: brak standardSVK.")
        version = int(svk.attrib.get("SVKVersion", "1"), 10)
        dep = int(svk.attrib.get("progDepChecked", "1"), 10)
        if not 0 <= version <= 255 or not 0 <= dep <= 255:
            raise ValueError(f"{name}: zła wersja SVK.")
        parts = [
            _part_from_node(child)
            for child in svk.iter()
            if _local(child.tag) == "partIdentification"
        ]
        if not parts or len(parts) > 80:
            raise ValueError(f"{name}: liczba części ma być od 1 do 80.")
        ecus.append(SvtEcu(addr, name, parts, version, dep))
    if not ecus:
        raise ValueError("SVT nie zawiera sterowników.")
    if len(ecus) > 40:
        raise ValueError("Za dużo sterowników (maks. 40).")
    return ecus


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


def _code_list(text: str, size: int, label: str) -> list[str]:
    pattern = _CODE4 if size == 4 else _CODE3
    codes: list[str] = []
    for raw in text.replace(",", " ").split():
        code = raw.strip().upper()
        if not pattern.match(code):
            raise ValueError(f"{label} {code!r} ma mieć {size} znaków A–Z 0–9.")
        codes.append(code)
    if len(codes) > 255:
        raise ValueError(f"Za dużo pozycji {label}.")
    return codes


def fa_from_fields(
    series: str,
    type_key: str,
    time_criteria: str,
    colour: str,
    fabric: str,
    e_text: str,
    sa_text: str,
    ho_text: str,
    vin: str = "",
) -> Fa:
    fa = Fa(
        series=_need4(series, "Seria"),
        type_key=_need4(type_key, "Typ"),
        time_criteria=_need4(time_criteria, "Data"),
        colour=_need4(colour, "Lakier"),
        fabric=_need4(fabric, "Tapicerka"),
        e_codes=_code_list(e_text, 4, "E-Wort"),
        sa_codes=_code_list(sa_text, 3, "SA"),
        ho_codes=_code_list(ho_text, 4, "HO"),
        vin=_vin_ok(vin) if vin.strip() else "",
    )
    if len(fa_bytes(fa)) > MAX_FA_BYTES:
        raise ValueError("FA jest za duże na emulator (maks. 480 bajtów).")
    return fa


def fa_from_bytes(raw: bytes) -> Fa:
    if len(raw) < 24 or raw[0] != 0x03:
        raise ValueError("To nie jest FA w wersji 3.")
    def text4(at: int) -> str:
        chunk = raw[at:at + 4]
        try:
            value = chunk.decode("ascii")
        except UnicodeDecodeError as exc:
            raise ValueError("FA zawiera znaki spoza ASCII.") from exc
        return _need4(value, "Pole FA")

    index = 21
    e_count = raw[index]
    index += 1
    if index + e_count * 4 > len(raw):
        raise ValueError("FA urywa się na E-Wortach.")
    e_codes = [text4(index + i * 4) for i in range(e_count)]
    index += e_count * 4
    if index >= len(raw):
        raise ValueError("FA urywa się przed SA.")
    sa_count = raw[index]
    index += 1
    if index + sa_count * 3 > len(raw):
        raise ValueError("FA urywa się na SA.")
    sa_codes = []
    for _ in range(sa_count):
        code = raw[index:index + 3].decode("ascii").upper()
        if not _CODE3.match(code):
            raise ValueError(f"SA {code!r} ma mieć 3 znaki.")
        sa_codes.append(code)
        index += 3
    if index >= len(raw):
        raise ValueError("FA urywa się przed HO.")
    ho_count = raw[index]
    index += 1
    if index + ho_count * 4 != len(raw):
        raise ValueError("Długość FA nie zgadza się z liczbą kodów.")
    ho_codes = [text4(index + i * 4) for i in range(ho_count)]
    return Fa(
        series=text4(1),
        type_key=text4(5),
        time_criteria=text4(9),
        colour=text4(13),
        fabric=text4(17),
        e_codes=e_codes,
        sa_codes=sa_codes,
        ho_codes=ho_codes,
    )


def save_fa(path: str | Path, fa: Fa) -> None:
    root = ET.Element("faList")
    fa_el = ET.SubElement(root, "fa")
    header = ET.SubElement(fa_el, "header")
    if fa.vin:
        header.set("vinLong", fa.vin)
    std = ET.SubElement(
        fa_el,
        "standardFA",
        series=fa.series,
        typeKey=fa.type_key,
        timeCriteria=fa.time_criteria,
        colourCode=fa.colour,
        fabricCode=fa.fabric,
        faVersion="3",
    )
    e_el = ET.SubElement(std, "eCodes")
    for code in fa.e_codes:
        ET.SubElement(e_el, "eCode").text = code
    sa_el = ET.SubElement(std, "saCodes")
    for code in fa.sa_codes:
        ET.SubElement(sa_el, "saCode").text = code
    ho_el = ET.SubElement(std, "hoCodes")
    for code in fa.ho_codes:
        ET.SubElement(ho_el, "hoCode").text = code
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def save_svt(path: str | Path, ecus: list[SvtEcu]) -> None:
    root = ET.Element("svt")
    ET.SubElement(root, "header", svtTypeInfo="IST")
    body = ET.SubElement(root, "standardSVT", svtVersion="5")
    for ecu in ecus:
        ecu_el = ET.SubElement(body, "ecu", baseVariant=ecu.name)
        addrs = ET.SubElement(ecu_el, "diagnosticAddresses")
        ET.SubElement(
            addrs,
            "diagnosticAddress",
            physicalOffset=str(ecu.addr),
            physicalOffsetAsHex=f"{ecu.addr:02X}",
        )
        svk = ET.SubElement(
            ecu_el,
            "standardSVK",
            SVKVersion=str(ecu.svk_version),
            progDepChecked=str(ecu.prog_dep),
        )
        for part in ecu.parts:
            node = ET.SubElement(svk, "partIdentification")
            ET.SubElement(node, "processClass").text = part.klass
            ET.SubElement(node, "id").text = part.ident
            ET.SubElement(node, "mainVersion").text = f"{part.main:03d}"
            ET.SubElement(node, "subVersion").text = f"{part.sub:03d}"
            ET.SubElement(node, "patchVersion").text = f"{part.patch:03d}"
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


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
    prefix = bytes.fromhex("03473032303556353131313139304333314b474e4c0141303930")
    if raw[: len(prefix)] != prefix or len(raw) != 208:
        raise SystemExit(f"FA.xml koduje się inaczej niż G20: {len(raw)} {raw[:26].hex()}")
    if fa.summary() != "G020 5V51 1119" or fa.vin != "WBA5V510X0FJ28775":
        raise SystemExit(f"{fa.summary()} {fa.vin}")
    baked = (here.parent / "src" / "g20_fa.inc").read_text()
    if "0x03, 0x47, 0x30, 0x32, 0x30" not in baked:
        raise SystemExit("g20_fa.inc nie zaczyna się od zamówienia G20")
    back = fa_from_bytes(raw)
    if fa_bytes(back) != raw:
        raise SystemExit("fa_from_bytes nie składa tego samego FA")
    svt = load_svt(here / "data" / "SVT.xml")
    first = svt[0].parts[0]
    if len(svt) != 29 or svt[0].addr != 0x10 or svt[0].name != "BDC_GW3":
        raise SystemExit(f"SVT {len(svt)} {svt[0]}")
    if first.klass != "HWEL" or first.ident != "00004159" or first.line() != "HWEL 00004159 013.000.002":
        raise SystemExit(first.line())
    if len(svt[0].wire()) != len(svt[0].parts) * 8:
        raise SystemExit("wire")
    parts = sum(len(ecu.parts) for ecu in svt)
    if parts > 360 or any(len(ecu.parts) > 80 for ecu in svt):
        raise SystemExit(f"SVT nie mieści się w emulatorze: {parts} części")
    import json
    import tempfile

    biggest = max(svt, key=lambda ecu: len(ecu.parts))
    payload = json.dumps(
        {
            "cmd": "svt_ecu",
            "addr": f"{biggest.addr:02X}",
            "name": biggest.name,
            "ver": str(biggest.svk_version),
            "dep": str(biggest.prog_dep),
            "hex": biggest.wire().hex(),
        },
        separators=(",", ":"),
    )
    if len(payload) > 1450:
        raise SystemExit(f"Największy sterownik nie mieści się w UDP: {len(payload)}")
    with tempfile.TemporaryDirectory() as tmp:
        fa_path = Path(tmp) / "FA.xml"
        svt_path = Path(tmp) / "SVT.xml"
        save_fa(fa_path, fa)
        save_svt(svt_path, svt)
        if fa_bytes(load_fa(fa_path)) != raw or load_fa(fa_path).vin != fa.vin:
            raise SystemExit("zapis FA.xml nie wraca")
        again = load_svt(svt_path)
        if [(e.addr, e.name, e.wire()) for e in again] != [(e.addr, e.name, e.wire()) for e in svt]:
            raise SystemExit("zapis SVT.xml nie wraca")
    print(f"ok {len(raw)} {fa.summary()} {fa.vin} svt {len(svt)} parts {parts} udp {len(payload)}")


if __name__ == "__main__":
    _self_check()
