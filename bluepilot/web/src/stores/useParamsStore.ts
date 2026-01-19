import { create } from "zustand";
import { paramsAPI } from "@/services/api";
import type { Parameter } from "@/types";

interface ParamsState {
  params: Record<string, Parameter>;
  stagedParams: Record<string, string | number | boolean>; // Staged (unsaved) values
  loading: boolean;
  error: string | null;
  searchQuery: string;

  // Actions
  fetchParams: () => Promise<void>;
  updateParam: (key: string, value: string | number | boolean) => Promise<void>;
  stageParam: (key: string, value: string | number | boolean) => void;
  unstageParam: (key: string) => void;
  clearStagedParams: () => void;
  applyStagedParams: () => void; // Apply staged values to local state after commit
  getEffectiveValue: (key: string) => string | number | boolean | null;
  getEffectiveParams: () => Record<string, Parameter>; // Get params with staged values applied
  setSearchQuery: (query: string) => void;
  getFilteredParams: () => Parameter[];
}

export const useParamsStore = create<ParamsState>((set, get) => ({
  params: {},
  stagedParams: {},
  loading: false,
  error: null,
  searchQuery: "",

  fetchParams: async () => {
    set({ loading: true, error: null });

    try {
      const params = await paramsAPI.getAll();
      const paramCount = Object.keys(params).length;
      console.log("[BluePilot] Fetched params:", paramCount, "parameters");

      // Debug: Log first param
      // const firstParam = Object.values(params)[0]
      // if (firstParam) {
      //   console.log('[BluePilot] Sample param:', firstParam)
      //   console.log('[BluePilot] Has type?', firstParam.type !== undefined)
      //   console.log('[BluePilot] Has last_modified?', firstParam.last_modified !== undefined)
      // }

      set({ params, loading: false });
    } catch (error) {
      console.error("Failed to fetch params:", error);
      set({
        error:
          error instanceof Error ? error.message : "Failed to fetch parameters",
        loading: false,
      });
    }
  },

  updateParam: async (key: string, value: string | number | boolean) => {
    try {
      await paramsAPI.update(key, value);

      set((state) => ({
        params: {
          ...state.params,
          [key]: {
            ...state.params[key],
            value,
          },
        },
      }));
    } catch (error) {
      set({
        error:
          error instanceof Error ? error.message : "Failed to update parameter",
      });
    }
  },

  stageParam: (key: string, value: string | number | boolean) => {
    set((state) => ({
      stagedParams: {
        ...state.stagedParams,
        [key]: value,
      },
    }));
  },

  unstageParam: (key: string) => {
    set((state) => {
      const newStagedParams = { ...state.stagedParams };
      delete newStagedParams[key];
      return { stagedParams: newStagedParams };
    });
  },

  clearStagedParams: () => {
    set({ stagedParams: {} });
  },

  applyStagedParams: () => {
    // After a successful commit, apply staged values to local params state
    set((state) => {
      const newParams = { ...state.params };
      for (const [key, value] of Object.entries(state.stagedParams)) {
        if (newParams[key]) {
          newParams[key] = { ...newParams[key], value };
        }
      }
      return { params: newParams, stagedParams: {} };
    });
  },

  getEffectiveValue: (key: string) => {
    const { params, stagedParams } = get();
    // Return staged value if it exists, otherwise return actual value
    if (key in stagedParams) {
      return stagedParams[key];
    }
    return params[key]?.value ?? null;
  },

  getEffectiveParams: () => {
    const { params, stagedParams } = get();
    // Return params with staged values overlaid
    if (Object.keys(stagedParams).length === 0) {
      return params;
    }
    const effectiveParams = { ...params };
    for (const [key, value] of Object.entries(stagedParams)) {
      if (effectiveParams[key]) {
        effectiveParams[key] = { ...effectiveParams[key], value };
      }
    }
    return effectiveParams;
  },

  setSearchQuery: (query: string) => {
    set({ searchQuery: query });
  },

  getFilteredParams: () => {
    const { params, searchQuery } = get();

    if (!searchQuery) {
      return Object.values(params);
    }

    const query = searchQuery.toLowerCase();
    return Object.values(params).filter((param) => {
      if (param.key.toLowerCase().includes(query)) {
        return true;
      }

      const valueString = (() => {
        if (typeof param.value === "string") return param.value;
        if (param.value === null || param.value === undefined) return "";
        try {
          return JSON.stringify(param.value);
        } catch {
          return String(param.value);
        }
      })().toLowerCase();

      if (valueString.includes(query)) {
        return true;
      }

      if (param.description?.toLowerCase().includes(query)) {
        return true;
      }

      if (param.decoded) {
        try {
          const decodedStr = JSON.stringify(param.decoded).toLowerCase();
          if (decodedStr.includes(query)) {
            return true;
          }
        } catch {
          // ignore JSON stringify issues
        }
      }

      return false;
    });
  },
}));
