import { useCallback, useEffect, useMemo, useState } from "preact/hooks";
import { api } from "../api";
import backSvg from "../assets/icons/back.svg?raw";
import houseSvg from "../assets/icons/house.svg?raw";
import { ErrorBannerStack, useErrorStack } from "../components/error-banner";
import { Icon } from "../components/icon";
import { useNavigate, usePath } from "../components/router";
import { formatArea, formatDistance, type DistanceUnit } from "../distance-units";
import { T, useI18n } from "../i18n";
import type { FloorplanInfo, HistoryFileInfo, MapData } from "../types";
import { normalizeError } from "../utils";
import { HistoryItemView } from "./history/item";

interface FloorplansViewProps {
    distanceUnit: DistanceUnit;
}

export function FloorplansView({ distanceUnit }: FloorplansViewProps) {
    const { t, formatDuration, formatNumber } = useI18n();
    const navigate = useNavigate();
    const path = usePath();
    const [errors, errorStack] = useErrorStack();
    const [floorplans, setFloorplans] = useState<FloorplanInfo[]>([]);
    const [loading, setLoading] = useState(true);
    const [selectedMap, setSelectedMap] = useState<MapData | null>(null);
    const [mapEmpty, setMapEmpty] = useState(false);
    const [busy, setBusy] = useState(false);

    const selectedSession = path.startsWith("/floorplans/") ? decodeURIComponent(path.slice(12)) : null;
    const selected = useMemo(
        () => (selectedSession ? (floorplans.find((f) => f.session === selectedSession) ?? null) : null),
        [floorplans, selectedSession],
    );

    const refresh = useCallback(async () => {
        const items = await api.getFloorplans();
        setFloorplans(items);
        return items;
    }, []);

    useEffect(() => {
        refresh()
            .catch((e: unknown) => errorStack.push(normalizeError(e, "Failed to load saved floorplans")))
            .finally(() => setLoading(false));
    }, [refresh, errorStack]);

    useEffect(() => {
        if (!selectedSession) {
            setSelectedMap(null);
            setMapEmpty(false);
            return;
        }

        setSelectedMap(null);
        setMapEmpty(false);

        api.getHistorySession(selectedSession)
            .then((maps) => {
                if (maps.length > 0) setSelectedMap(maps[0]);
                else setMapEmpty(true);
            })
            .catch((e: unknown) => errorStack.push(normalizeError(e, "Failed to load floorplan map")));
    }, [selectedSession, errorStack]);

    const handleBack = useCallback(() => {
        if (selectedSession) {
            navigate("/floorplans");
            errorStack.clear();
        } else {
            navigate("/");
        }
    }, [selectedSession, navigate, errorStack]);

    const rename = useCallback(
        async (floorplan: FloorplanInfo) => {
            const nextName = window.prompt(t("Floorplan name"), floorplan.name);
            if (nextName === null) return;
            const trimmed = nextName.trim();
            if (!trimmed || trimmed === floorplan.name) return;

            setBusy(true);
            try {
                await api.saveFloorplan(floorplan.session, trimmed);
                await refresh();
            } catch (e: unknown) {
                errorStack.push(normalizeError(e, "Failed to rename floorplan"));
            } finally {
                setBusy(false);
            }
        },
        [refresh, errorStack, t],
    );

    const unpin = useCallback(
        async (floorplan: FloorplanInfo) => {
            if (!window.confirm(t("Remove this saved floorplan? The cleaning history data will be kept."))) return;

            setBusy(true);
            try {
                await api.deleteFloorplan(floorplan.session);
                await refresh();
                if (selectedSession === floorplan.session) navigate("/floorplans");
            } catch (e: unknown) {
                errorStack.push(normalizeError(e, "Failed to remove floorplan"));
            } finally {
                setBusy(false);
            }
        },
        [refresh, selectedSession, navigate, errorStack, t],
    );

    const historyFile = useMemo<HistoryFileInfo | null>(() => {
        if (!selected) return null;

        return {
            name: selected.session,
            size: selected.size,
            compressed: selected.compressed,
            recording: false,
            session: selectedMap?.session ?? null,
            summary: selected.summary ?? selectedMap?.summary ?? null,
        };
    }, [selected, selectedMap]);

    return (
        <>
            <div class="header">
                <button type="button" class="header-back-btn" onClick={handleBack} aria-label={t("Back")}>
                    <Icon svg={backSvg} />
                </button>
                <h1>{selected ? selected.name : t("Saved Floorplans")}</h1>
                <div class="header-right-spacer" />
            </div>

            <ErrorBannerStack errors={errors} />

            <div class="history-page">
                {loading && (
                    <div class="history-empty">
                        <T>Loading...</T>
                    </div>
                )}

                {!loading && !selectedSession && (
                    <>
                        <div class="history-summary">
                            <span>
                                {floorplans.length} {t(floorplans.length === 1 ? "floorplan" : "floorplans")}
                            </span>
                            <span class="floorplan-summary-note">
                                <T>Saved maps are protected from automatic history cleanup</T>
                            </span>
                        </div>

                        {floorplans.length === 0 && (
                            <div class="history-empty">
                                <T>No saved floorplans yet. Open Cleaning History to save a completed map.</T>
                            </div>
                        )}

                        {floorplans.map((floorplan) => (
                            <div class="history-session-row" key={floorplan.session}>
                                <button
                                    type="button"
                                    class="history-session-card"
                                    onClick={() => navigate(`/floorplans/${encodeURIComponent(floorplan.session)}`)}
                                >
                                    <div class="history-session-icon">
                                        <Icon svg={houseSvg} />
                                    </div>
                                    <div class="history-session-body">
                                        <div class="history-session-header">
                                            <span class="history-session-mode">{floorplan.name}</span>
                                            <span class="history-running-badge">
                                                <T>Saved</T>
                                            </span>
                                        </div>
                                        {floorplan.summary && (
                                            <div class="history-session-stats">
                                                <span>{formatDuration(floorplan.summary.duration)}</span>
                                                <span>
                                                    {formatDistance(
                                                        floorplan.summary.distanceTraveled,
                                                        distanceUnit,
                                                        formatNumber,
                                                    )}
                                                </span>
                                                <span>
                                                    {formatArea(
                                                        floorplan.summary.areaCovered,
                                                        distanceUnit,
                                                        formatNumber,
                                                    )}
                                                </span>
                                            </div>
                                        )}
                                    </div>
                                    <span class="history-session-chevron">&rsaquo;</span>
                                </button>
                                <div class="floorplan-row-actions">
                                    <button
                                        type="button"
                                        class="floorplan-row-btn"
                                        onClick={() => rename(floorplan)}
                                        disabled={busy}
                                    >
                                        <T>Rename</T>
                                    </button>
                                    <button
                                        type="button"
                                        class="floorplan-row-btn danger"
                                        onClick={() => unpin(floorplan)}
                                        disabled={busy}
                                    >
                                        <T>Unpin</T>
                                    </button>
                                </div>
                            </div>
                        ))}
                    </>
                )}

                {!loading && selected && historyFile && (
                    <>
                        <div class="floorplan-detail-actions">
                            <span class="floorplan-detail-name">{selected.name}</span>
                            <button
                                type="button"
                                class="floorplan-row-btn"
                                onClick={() => rename(selected)}
                                disabled={busy}
                            >
                                <T>Rename</T>
                            </button>
                            <button
                                type="button"
                                class="floorplan-row-btn danger"
                                onClick={() => unpin(selected)}
                                disabled={busy}
                            >
                                <T>Unpin</T>
                            </button>
                        </div>
                        <HistoryItemView
                            file={historyFile}
                            map={selectedMap}
                            mapEmpty={mapEmpty}
                            recording={false}
                            distanceUnit={distanceUnit}
                        />
                    </>
                )}
            </div>
        </>
    );
}
