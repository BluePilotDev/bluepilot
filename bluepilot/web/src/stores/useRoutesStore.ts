import { create } from "zustand";
import { routesAPI } from "@/services/api";
import type { Route, RouteDetails } from "@/types";

interface RoutesState {
  routes: Route[];
  selectedRoute: RouteDetails | null;
  loading: boolean;
  error: string | null;
  page: number;
  hasMore: boolean;

  // Actions
  fetchRoutes: (page?: number) => Promise<void>;
  fetchRouteDetails: (routeId: string) => Promise<RouteDetails | null>;
  deleteRoute: (routeId: string) => Promise<void>;
  preserveRoute: (routeId: string) => Promise<void>;
  clearCache: () => Promise<void>;
  reset: () => void;
}

const initialState = {
  routes: [],
  selectedRoute: null,
  loading: false,
  error: null,
  page: 1,
  hasMore: true,
};

export const useRoutesStore = create<RoutesState>((set) => ({
  ...initialState,

  fetchRoutes: async (page = 1) => {
    set({ loading: true, error: null });

    try {
      const response = await routesAPI.getAll(page, 50);

      // Handle different response formats
      let routes: Route[] = [];
      if (Array.isArray(response)) {
        routes = response;
      } else if (
        response &&
        typeof response === "object" &&
        "routes" in response
      ) {
        routes = (response as any).routes || [];
      } else if (
        response &&
        typeof response === "object" &&
        "data" in response
      ) {
        routes = (response as any).data || [];
      }

      // Normalize routes: ensure baseName is set (backend uses baseName as primary identifier)
      routes = routes.map((route) => {
        const baseName = route.baseName || route.id || "";
        return {
          ...route,
          baseName,
          id: route.id || baseName, // Ensure id is also set for compatibility
        };
      });

      console.log("Fetched routes:", routes);
      // console.log('First route baseName:', routes[0]?.baseName)

      set((state) => ({
        routes: page === 1 ? routes : [...state.routes, ...routes],
        page,
        hasMore: routes.length === 50,
        loading: false,
      }));
    } catch (error) {
      console.error("Failed to fetch routes:", error);
      set({
        error:
          error instanceof Error ? error.message : "Failed to fetch routes",
        loading: false,
      });
    }
  },

  fetchRouteDetails: async (routeId: string) => {
    set({ loading: true, error: null });

    try {
      const route = await routesAPI.getOne(routeId);
      set({ selectedRoute: route, loading: false });
      return route;
    } catch (error) {
      set({
        error:
          error instanceof Error
            ? error.message
            : "Failed to fetch route details",
        loading: false,
      });
      return null;
    }
  },

  deleteRoute: async (routeId: string) => {
    try {
      await routesAPI.delete(routeId);

      set((state) => ({
        routes: state.routes.filter(
          (r) => r.baseName !== routeId && r.id !== routeId
        ),
        selectedRoute:
          state.selectedRoute?.baseName === routeId ||
          state.selectedRoute?.id === routeId
            ? null
            : state.selectedRoute,
      }));
    } catch (error) {
      set({
        error:
          error instanceof Error ? error.message : "Failed to delete route",
      });
    }
  },

  preserveRoute: async (routeId: string) => {
    try {
      await routesAPI.preserve(routeId);

      set((state) => ({
        routes: state.routes.map((r) =>
          r.baseName === routeId || r.id === routeId
            ? { ...r, preserved: !r.preserved }
            : r
        ),
      }));
    } catch (error: any) {
      // Extract error message from axios response
      const errorMessage =
        error?.response?.data?.error ||
        error?.response?.data?.message ||
        error?.message ||
        "Failed to preserve route";

      console.error("Preserve route error:", {
        routeId,
        status: error?.response?.status,
        data: error?.response?.data,
        message: errorMessage,
      });

      set({
        error: errorMessage,
      });

      // Re-throw so calling code can handle it
      throw new Error(errorMessage);
    }
  },

  clearCache: async () => {
    try {
      await routesAPI.clearCache();
    } catch (error) {
      set({
        error: error instanceof Error ? error.message : "Failed to clear cache",
      });
    }
  },

  reset: () => set(initialState),
}));
