#!/usr/bin/env python3
# Apply WPA3-Personal / SAE userspace-SME bring-up changes to the MediaTek
# WLAN gen2 driver used by grandppltedx.
#
# Run from the kernel root:
#     python3 apply_wpa3_sae_stage3.py
#
# Options:
#     --root PATH
#     --check
#     --backup
#     --no-diff
#
# NOTE: This is an experimental bring-up patch. Adding cfg80211_ops::auth
# makes nl80211/wpa_supplicant treat the driver as userspace-SME capable.
# WPA/WPA2 behavior may change until the association/deauth path is fully
# adapted.

from __future__ import annotations

import argparse
import re
import shutil
import subprocess
import sys
from pathlib import Path


BASE = Path("drivers/misc/mediatek/connectivity/wlan/gen2")

FILES = {
    "gl_init": BASE / "os/linux/gl_init.c",
    "gl_cfg_h": BASE / "os/linux/include/gl_cfg80211.h",
    "gl_cfg_c": BASE / "os/linux/gl_cfg80211.c",
    "gl_kal_h": BASE / "os/linux/include/gl_kal.h",
    "gl_kal_c": BASE / "os/linux/gl_kal.c",
    "mac_h": BASE / "include/nic/mac.h",
    "saa_fsm": BASE / "mgmt/saa_fsm.c",
}


class PatchError(RuntimeError):
    pass


def replace_once(text: str, old: str, new: str, label: str) -> tuple[str, bool]:
    if new in text:
        print(f"[skip] {label}: already applied")
        return text, False
    count = text.count(old)
    if count != 1:
        raise PatchError(
            f"{label}: expected exactly one anchor, found {count}\n"
            f"Anchor starts with:\n{old[:240]}"
        )
    return text.replace(old, new, 1), True


def insert_after_once(text: str, anchor: str, addition: str, label: str) -> tuple[str, bool]:
    if addition.strip() in text:
        print(f"[skip] {label}: already applied")
        return text, False
    count = text.count(anchor)
    if count != 1:
        raise PatchError(f"{label}: expected exactly one anchor, found {count}")
    return text.replace(anchor, anchor + addition, 1), True


def ensure_stage1_sae_feature(text: str) -> tuple[str, bool]:
    if "prWiphy->features |= NL80211_FEATURE_SAE;" in text:
        print("[skip] gl_init.c: NL80211_FEATURE_SAE already advertised")
        return text, False

    anchor = (
        "\tprWiphy->flags = WIPHY_FLAG_SUPPORTS_FW_ROAM\n"
        "\t\t| WIPHY_FLAG_HAS_REMAIN_ON_CHANNEL\n"
        "\t\t| WIPHY_FLAG_SUPPORTS_SCHED_SCAN;\n"
    )
    addition = (
        "\n"
        "\t/* Allow wpa_supplicant to use SAE/WPA3-Personal. */\n"
        "\tprWiphy->features |= NL80211_FEATURE_SAE;\n"
    )
    return insert_after_once(
        text, anchor, addition,
        "gl_init.c: advertise NL80211_FEATURE_SAE"
    )


AUTH_IMPL = r"""
/*----------------------------------------------------------------------------*/
/*!
 * @brief Userspace-SME authentication hook.
 *
 * SAE is calculated by wpa_supplicant. cfg80211 passes the Authentication
 * transaction/status fields and the SAE Commit/Confirm payload through
 * req->sae_data. The driver only transports the resulting Authentication
 * management frame.
 */
/*----------------------------------------------------------------------------*/
int mtk_cfg80211_auth(struct wiphy *wiphy,
		      struct net_device *ndev,
		      struct cfg80211_auth_request *req)
{
	struct wireless_dev *wdev;
	struct ieee80211_mgmt *mgmt;
	struct cfg80211_mgmt_tx_params params;
	PUINT_8 pucFrame;
	size_t u4AuthDataOffset;
	size_t u4FrameLen;
	UINT_64 u8Cookie = 0;
	INT_32 i4Ret;

	if (!wiphy || !ndev || !req || !req->bss)
		return -EINVAL;

	if (req->auth_type != NL80211_AUTHTYPE_SAE) {
		DBGLOG(REQ, WARN, "userspace auth unsupported type=%d\n",
		       req->auth_type);
		return -EOPNOTSUPP;
	}

	/*
	 * For SAE, cfg80211 requires sae_data to contain at least:
	 *   2 bytes Authentication transaction sequence
	 *   2 bytes Status code
	 */
	if (!req->sae_data || req->sae_data_len < 4) {
		DBGLOG(REQ, ERROR, "SAE auth missing data, len=%zu\n",
		       req->sae_data_len);
		return -EINVAL;
	}

	wdev = ndev->ieee80211_ptr;
	if (!wdev)
		return -EINVAL;

	/*
	 * Copy sae_data directly starting at auth_transaction:
	 *
	 *   auth_transaction (2)
	 *   status_code      (2)
	 *   SAE Commit/Confirm payload
	 *
	 * Extra authentication IEs, if any, are appended afterwards.
	 */
	u4AuthDataOffset =
		offsetof(struct ieee80211_mgmt, u.auth.auth_transaction);
	u4FrameLen = u4AuthDataOffset + req->sae_data_len + req->ie_len;

	pucFrame = kzalloc(u4FrameLen, GFP_KERNEL);
	if (!pucFrame)
		return -ENOMEM;

	mgmt = (struct ieee80211_mgmt *)pucFrame;

	mgmt->frame_control =
		cpu_to_le16(IEEE80211_FTYPE_MGMT | IEEE80211_STYPE_AUTH);
	COPY_MAC_ADDR(mgmt->da, req->bss->bssid);
	COPY_MAC_ADDR(mgmt->sa, ndev->dev_addr);
	COPY_MAC_ADDR(mgmt->bssid, req->bss->bssid);
	mgmt->u.auth.auth_alg = cpu_to_le16(WLAN_AUTH_SAE);

	kalMemCopy(pucFrame + u4AuthDataOffset,
		   req->sae_data,
		   req->sae_data_len);

	if (req->ie && req->ie_len)
		kalMemCopy(pucFrame + u4AuthDataOffset + req->sae_data_len,
			   req->ie,
			   req->ie_len);

	kalMemZero(&params, sizeof(params));
	params.chan = req->bss->channel;
	params.buf = pucFrame;
	params.len = u4FrameLen;
	params.no_cck = FALSE;
	params.dont_wait_for_ack = FALSE;

	DBGLOG(REQ, INFO,
	       "SAE TX auth BSSID=%pM trans=%u status=%u len=%zu\n",
	       req->bss->bssid,
	       le16_to_cpu(mgmt->u.auth.auth_transaction),
	       le16_to_cpu(mgmt->u.auth.status_code),
	       u4FrameLen);

	i4Ret = mtk_cfg80211_mgmt_tx(wiphy, wdev, &params, &u8Cookie);

	DBGLOG(REQ, INFO, "SAE TX auth result=%d\n", i4Ret);

	kfree(pucFrame);
	return i4Ret;
}

"""


RX_MLME_IMPL = r"""
VOID kalIndicateRxMlmeFrame(IN P_GLUE_INFO_T prGlueInfo,
			    IN P_SW_RFB_T prSwRfb)
{
	struct wireless_dev *wdev;

	if (!prGlueInfo || !prSwRfb || !prGlueInfo->prDevHandler)
		return;

	wdev = prGlueInfo->prDevHandler->ieee80211_ptr;
	if (!wdev)
		return;

	DBGLOG(AIS, INFO, "RX MLME frame fc=0x%x len=%u\n",
	       ((P_WLAN_MAC_HEADER_T) prSwRfb->pvHeader)->u2FrameCtrl,
	       prSwRfb->u2PacketLen);

	/*
	 * cfg80211_rx_mlme_mgmt() may sleep and requires the wdev mutex.
	 * RX processing here runs in the driver's RX thread context.
	 */
	mutex_lock(&wdev->mtx);
	cfg80211_rx_mlme_mgmt(prGlueInfo->prDevHandler,
			      prSwRfb->pvHeader,
			      prSwRfb->u2PacketLen);
	mutex_unlock(&wdev->mtx);
}

"""


def patch_gl_init(text: str) -> tuple[str, bool]:
    changed = False
    text, c = ensure_stage1_sae_feature(text)
    changed |= c

    if ".auth = mtk_cfg80211_auth," in text:
        print("[skip] gl_init.c: .auth already registered")
    else:
        anchor = "\t.scan = mtk_cfg80211_scan,\n\t.connect = mtk_cfg80211_connect,"
        replacement = (
            "\t.scan = mtk_cfg80211_scan,\n"
            "\t.auth = mtk_cfg80211_auth,\n"
            "\t.connect = mtk_cfg80211_connect,"
        )
        text, c = replace_once(
            text, anchor, replacement,
            "gl_init.c: register cfg80211 .auth"
        )
        changed |= c

    return text, changed


def patch_gl_cfg_h(text: str) -> tuple[str, bool]:
    proto = (
        "int mtk_cfg80211_auth(struct wiphy *wiphy,\n"
        "\t\t      struct net_device *ndev,\n"
        "\t\t      struct cfg80211_auth_request *req);\n\n"
    )
    if "int mtk_cfg80211_auth(" in text:
        print("[skip] gl_cfg80211.h: auth prototype already present")
        return text, False

    anchor = "int mtk_cfg80211_scan(struct wiphy *wiphy, struct cfg80211_scan_request *request);\n\n"
    return insert_after_once(
        text, anchor, proto,
        "gl_cfg80211.h: add auth prototype"
    )


def patch_gl_cfg_c(text: str) -> tuple[str, bool]:
    if "int mtk_cfg80211_auth(" in text:
        print("[skip] gl_cfg80211.c: auth implementation already present")
        return text, False

    anchor = "static UINT_8 wepBuf[48];\n"
    count = text.count(anchor)
    if count != 1:
        raise PatchError(
            f"gl_cfg80211.c: expected one 'static UINT_8 wepBuf[48];', found {count}"
        )
    return text.replace(anchor, AUTH_IMPL + anchor, 1), True


def patch_gl_kal_h(text: str) -> tuple[str, bool]:
    proto = (
        "\nVOID kalIndicateRxMlmeFrame(IN P_GLUE_INFO_T prGlueInfo,\n"
        "\t\t\t    IN P_SW_RFB_T prSwRfb);\n"
    )
    if "kalIndicateRxMlmeFrame(" in text:
        print("[skip] gl_kal.h: MLME RX prototype already present")
        return text, False

    anchor = "VOID kalIndicateRxMgmtFrame(IN P_GLUE_INFO_T prGlueInfo, IN P_SW_RFB_T prSwRfb);\n"
    return insert_after_once(
        text, anchor, proto,
        "gl_kal.h: add MLME RX prototype"
    )


def patch_gl_kal_c(text: str) -> tuple[str, bool]:
    if "VOID kalIndicateRxMlmeFrame(" in text:
        print("[skip] gl_kal.c: MLME RX helper already present")
        return text, False

    anchors = [
        "}\t\t\t\t/* kalIndicateRxMgmtFrame */\n",
        "} /* kalIndicateRxMgmtFrame */\n",
    ]
    anchor = None
    for candidate in anchors:
        if text.count(candidate) == 1:
            anchor = candidate
            break

    if anchor is None:
        raise PatchError(
            "gl_kal.c: could not uniquely find the end of kalIndicateRxMgmtFrame()"
        )

    return text.replace(anchor, anchor + RX_MLME_IMPL, 1), True


def patch_mac_h(text: str) -> tuple[str, bool]:
    if "AUTH_ALGORITHM_NUM_SAE" in text:
        print("[skip] mac.h: AUTH_ALGORITHM_NUM_SAE already defined")
        return text, False

    anchor = (
        "#define AUTH_ALGORITHM_NUM_OPEN_SYSTEM              0\t/* Open System */\n"
        "#define AUTH_ALGORITHM_NUM_SHARED_KEY               1\t/* Shared Key */\n"
        "#define AUTH_ALGORITHM_NUM_FAST_BSS_TRANSITION      2\t/* Fast BSS Transition */\n"
    )
    replacement = anchor + (
        "#define AUTH_ALGORITHM_NUM_SAE                      3\t/* SAE */\n"
    )
    return replace_once(
        text, anchor, replacement,
        "mac.h: add SAE authentication algorithm number"
    )


def patch_saa_fsm(text: str) -> tuple[str, bool]:
    marker = "SAE is handled by wpa_supplicant through cfg80211 userspace SME."
    if marker in text:
        print("[skip] saa_fsm.c: SAE RX interception already present")
        return text, False

    # There may be both a forward declaration/prototype and the real function.
    # Select only the occurrence whose signature is followed by '{' before ';'.
    name_matches = list(re.finditer(r"\bsaaFsmRunEventRxAuth\s*\(", text))
    definitions = []

    for m in name_matches:
        pos = m.end()
        paren_depth = 1

        # Find the closing ')' of the function parameter list.
        while pos < len(text) and paren_depth:
            ch = text[pos]
            if ch == "(":
                paren_depth += 1
            elif ch == ")":
                paren_depth -= 1
            pos += 1

        if paren_depth != 0:
            continue

        # Skip whitespace/comments between ')' and '{' or ';'.
        scan = pos
        while scan < len(text):
            if text[scan].isspace():
                scan += 1
                continue

            if text.startswith("/*", scan):
                end_comment = text.find("*/", scan + 2)
                if end_comment < 0:
                    break
                scan = end_comment + 2
                continue

            if text.startswith("//", scan):
                end_line = text.find("\n", scan + 2)
                if end_line < 0:
                    scan = len(text)
                    break
                scan = end_line + 1
                continue

            break

        if scan < len(text) and text[scan] == "{":
            definitions.append((m.start(), scan))

    if len(definitions) != 1:
        raise PatchError(
            f"saa_fsm.c: found {len(name_matches)} name occurrence(s), "
            f"but expected exactly one function definition; found "
            f"{len(definitions)} definition(s)"
        )

    fn_start, body_open = definitions[0]

    # Limit edits to the beginning of the real function body.
    # Finding the first switch is convenient in this legacy state machine.
    search_end = text.find("switch", body_open)
    if search_end < 0 or search_end - body_open > 4000:
        search_end = min(len(text), body_open + 4000)

    prologue = text[body_open:search_end]

    # Add Authentication-frame pointer next to the STA record declaration.
    decl_re = re.compile(
        r"(?P<indent>^[ \t]*)P_STA_RECORD_T[ \t]+prStaRec;[ \t]*$",
        re.MULTILINE,
    )
    decl_match = decl_re.search(prologue)
    if not decl_match:
        raise PatchError(
            "saa_fsm.c: P_STA_RECORD_T prStaRec declaration not found "
            "inside the real saaFsmRunEventRxAuth() definition"
        )

    indent = decl_match.group("indent")
    decl_abs_end = body_open + decl_match.end()

    text = (
        text[:decl_abs_end]
        + "\n"
        + indent
        + "P_WLAN_AUTH_FRAME_T prAuthFrame;"
        + text[decl_abs_end:]
    )

    # The insertion changed offsets. Locate the definition again.
    name_matches = list(re.finditer(r"\bsaaFsmRunEventRxAuth\s*\(", text))
    definitions = []

    for m in name_matches:
        pos = m.end()
        paren_depth = 1
        while pos < len(text) and paren_depth:
            ch = text[pos]
            if ch == "(":
                paren_depth += 1
            elif ch == ")":
                paren_depth -= 1
            pos += 1

        if paren_depth:
            continue

        scan = pos
        while scan < len(text):
            if text[scan].isspace():
                scan += 1
                continue
            if text.startswith("/*", scan):
                end_comment = text.find("*/", scan + 2)
                if end_comment < 0:
                    break
                scan = end_comment + 2
                continue
            if text.startswith("//", scan):
                end_line = text.find("\n", scan + 2)
                if end_line < 0:
                    break
                scan = end_line + 1
                continue
            break

        if scan < len(text) and text[scan] == "{":
            definitions.append((m.start(), scan))

    if len(definitions) != 1:
        raise PatchError(
            "saa_fsm.c: could not uniquely relocate function definition "
            "after adding prAuthFrame"
        )

    fn_start, body_open = definitions[0]
    search_end = text.find("switch", body_open)
    if search_end < 0 or search_end - body_open > 4500:
        search_end = min(len(text), body_open + 4500)
    prologue = text[body_open:search_end]

    assert_re = re.compile(
        r"(?P<indent>^[ \t]*)ASSERT\s*\(\s*prSwRfb\s*\)\s*;[ \t]*(?:\r?\n)?",
        re.MULTILINE,
    )
    assert_match = assert_re.search(prologue)
    if not assert_match:
        raise PatchError(
            "saa_fsm.c: ASSERT(prSwRfb) not found inside "
            "the real saaFsmRunEventRxAuth() definition"
        )

    # Accept line-wrapped cnmGetStaRecByIndex() formatting.
    assign_re = re.compile(
        r"^[ \t]*prStaRec\s*=\s*cnmGetStaRecByIndex\s*"
        r"\(\s*prAdapter\s*,\s*prSwRfb->ucStaRecIdx\s*\)\s*;[ \t]*",
        re.MULTILINE,
    )
    assign_match = assign_re.search(prologue, assert_match.end())
    if not assign_match:
        raise PatchError(
            "saa_fsm.c: initial prStaRec = cnmGetStaRecByIndex(...) "
            "not found inside the real saaFsmRunEventRxAuth() definition"
        )

    indent = assert_match.group("indent")

    replacement = (
        f"{indent}ASSERT(prSwRfb);\n"
        f"{indent}if (!prSwRfb || !prSwRfb->pvHeader)\n"
        f"{indent}\treturn;\n"
        "\n"
        f"{indent}prAuthFrame = (P_WLAN_AUTH_FRAME_T) prSwRfb->pvHeader;\n"
        "\n"
        f"{indent}/* SAE is handled by wpa_supplicant through cfg80211 userspace SME. */\n"
        f"{indent}if (prSwRfb->u2PacketLen >= offsetof(WLAN_AUTH_FRAME_T, aucInfoElem) &&\n"
        f"{indent}    prAuthFrame->u2AuthAlgNum == AUTH_ALGORITHM_NUM_SAE) {{\n"
        f"{indent}\tDBGLOG(SAA, INFO,\n"
        f"{indent}\t       \"SAE RX auth from %pM trans=%u status=%u len=%u\\n\",\n"
        f"{indent}\t       prAuthFrame->aucSrcAddr,\n"
        f"{indent}\t       prAuthFrame->u2AuthTransSeqNo,\n"
        f"{indent}\t       prAuthFrame->u2StatusCode,\n"
        f"{indent}\t       prSwRfb->u2PacketLen);\n"
        "\n"
        f"{indent}\tkalIndicateRxMlmeFrame(prAdapter->prGlueInfo, prSwRfb);\n"
        f"{indent}\treturn;\n"
        f"{indent}}}\n"
        "\n"
        f"{indent}prStaRec = cnmGetStaRecByIndex(prAdapter, prSwRfb->ucStaRecIdx);"
    )

    replace_start = body_open + assert_match.start()
    replace_end = body_open + assign_match.end()

    text = text[:replace_start] + replacement + text[replace_end:]

    print("[patch] saa_fsm.c: route SAE Authentication RX to cfg80211")
    return text, True


PATCHERS = {
    "gl_init": patch_gl_init,
    "gl_cfg_h": patch_gl_cfg_h,
    "gl_cfg_c": patch_gl_cfg_c,
    "gl_kal_h": patch_gl_kal_h,
    "gl_kal_c": patch_gl_kal_c,
    "mac_h": patch_mac_h,
    "saa_fsm": patch_saa_fsm,
}


def run_git_diff(root: Path, paths: list[Path]) -> None:
    try:
        cp = subprocess.run(
            ["git", "diff", "--"] + [str(p) for p in paths],
            cwd=root,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )
    except FileNotFoundError:
        return

    if cp.stdout:
        print("\n========== git diff ==========\n")
        print(cp.stdout, end="")
    elif cp.stderr:
        print(cp.stderr, file=sys.stderr)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=Path("."))
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--backup", action="store_true")
    parser.add_argument("--no-diff", action="store_true")
    args = parser.parse_args()

    root = args.root.resolve()

    missing = [rel for rel in FILES.values() if not (root / rel).is_file()]
    if missing:
        print("ERRO: não parece ser a raiz correta do kernel.", file=sys.stderr)
        for rel in missing:
            print(f"  ausente: {rel}", file=sys.stderr)
        return 2

    print(f"Kernel root: {root}")
    print("WPA3 SAE stage3: validating source anchors...")

    staged: dict[Path, str] = {}
    changed_paths: list[Path] = []

    try:
        for key, rel in FILES.items():
            path = root / rel
            original = path.read_text(encoding="utf-8")
            patched, changed = PATCHERS[key](original)
            staged[path] = patched
            if changed:
                changed_paths.append(rel)
    except PatchError as e:
        print(f"\nERRO: {e}", file=sys.stderr)
        print("Nenhum arquivo foi gravado.", file=sys.stderr)
        return 3

    if args.check:
        if changed_paths:
            print("\n[check] patch pode ser aplicado. Arquivos que mudariam:")
            for p in changed_paths:
                print(f"  {p}")
        else:
            print("\n[check] todas as alterações já parecem aplicadas.")
        return 0

    if not changed_paths:
        print("\nNada para alterar; patch já parece aplicado.")
        return 0

    if args.backup:
        for rel in changed_paths:
            src = root / rel
            dst = Path(str(src) + ".wpa3-sae.bak")
            if not dst.exists():
                shutil.copy2(src, dst)
                print(f"[backup] {dst.relative_to(root)}")

    # Validate all anchors first; only now write files.
    for rel in changed_paths:
        path = root / rel
        path.write_text(staged[path], encoding="utf-8")
        print(f"[write] {rel}")

    print("\nPatch aplicado.")
    print("ATENÇÃO: isto é bring-up de userspace SME/SAE.")
    print("A associação pós-SAE ainda pode precisar de ajuste em mtk_cfg80211_assoc().")
    print("Adicionar .auth também pode mudar temporariamente o comportamento WPA/WPA2.")

    if not args.no_diff:
        run_git_diff(root, changed_paths)

    print("\nTeste sugerido depois de compilar e bootar:")
    print("  adb root")
    print("  adb logcat -c")
    print("  # tente conectar numa rede WPA3-Personal/SAE-only")
    print("  adb shell dmesg | grep -iE 'SAE TX|SAE RX|RX MLME|auth|assoc'")
    print("  adb logcat -b all -d -v threadtime | grep -iE 'SAE|authenticate|association|nl80211'")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
