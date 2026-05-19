from __future__ import annotations

from math import cos, pi, sin, sqrt
from typing import Optional

from sdk._dependencies import require_cadquery

from .cadquery_helpers import _mesh_geometry_from_cadquery_model
from .common import _EPS
from .primitives import MeshGeometry, _adopt_mesh_geometry, _mesh_geometry_shifted_to_z0
from .specs import FanRotorBlade, FanRotorHub, FanRotorShroud


class FanRotorGeometry(MeshGeometry):
    """
    Build an axial fan rotor with a central hub and pitched blades.
    """

    def __init__(
        self,
        outer_radius: float,
        hub_radius: float,
        blade_count: int,
        *,
        thickness: float,
        blade_pitch_deg: float = 28.0,
        blade_sweep_deg: float = 20.0,
        blade_root_chord: Optional[float] = None,
        blade_tip_chord: Optional[float] = None,
        blade: Optional[FanRotorBlade] = None,
        hub: Optional[FanRotorHub] = None,
        shroud: Optional[FanRotorShroud] = None,
        center: bool = True,
    ):
        super().__init__()
        outer_radius = float(outer_radius)
        hub_radius = float(hub_radius)
        thickness = float(thickness)
        blade_pitch_deg = float(blade_pitch_deg)
        blade_sweep_deg = float(blade_sweep_deg)
        blade_count = int(blade_count)
        blade = blade or FanRotorBlade()
        hub = hub or FanRotorHub()

        if outer_radius <= 0.0 or hub_radius <= 0.0 or thickness <= 0.0:
            raise ValueError("outer_radius, hub_radius, and thickness must be positive")
        if blade_count < 2:
            raise ValueError("blade_count must be at least 2")
        if hub_radius >= outer_radius:
            raise ValueError("hub_radius must be less than outer_radius")
        if abs(blade_pitch_deg) >= 85.0:
            raise ValueError("abs(blade_pitch_deg) must be < 85")
        if abs(blade_sweep_deg) >= 85.0:
            raise ValueError("abs(blade_sweep_deg) must be < 85")

        radial_span = outer_radius - hub_radius
        root_chord = (
            float(blade_root_chord)
            if blade_root_chord is not None
            else max(radial_span * 0.32, thickness * 1.5)
        )
        tip_chord = (
            float(blade_tip_chord)
            if blade_tip_chord is not None
            else max(radial_span * 0.20, thickness * 1.2)
        )
        if root_chord <= 0.0 or tip_chord <= 0.0:
            raise ValueError("blade_root_chord and blade_tip_chord must be positive")
        if max(root_chord, tip_chord) >= outer_radius * 1.6:
            raise ValueError("blade chords are too large for the rotor envelope")

        blade_shape = str(blade.shape)
        if blade_shape not in {"straight", "scimitar", "broad", "narrow"}:
            raise ValueError("blade.shape must be one of straight, scimitar, broad, or narrow")
        hub_style = str(hub.style)
        if hub_style not in {"flat", "domed", "capped", "spinner"}:
            raise ValueError("hub.style must be one of flat, domed, capped, or spinner")

        tip_pitch_deg = (
            float(blade.tip_pitch_deg)
            if blade.tip_pitch_deg is not None
            else blade_pitch_deg * 0.56
        )
        if abs(tip_pitch_deg) >= 85.0:
            raise ValueError("abs(blade.tip_pitch_deg) must be < 85")
        camber = float(blade.camber)
        if abs(camber) > 0.50:
            raise ValueError("abs(blade.camber) must be <= 0.5")
        tip_clearance = float(blade.tip_clearance)
        if tip_clearance < 0.0:
            raise ValueError("blade.tip_clearance must be non-negative")

        shroud_thickness = 0.0
        shroud_depth = 0.0
        ring_inner_radius: Optional[float] = None
        ring_outer_radius: Optional[float] = None
        shroud_lip_depth = 0.0
        if shroud is not None:
            shroud_thickness = float(shroud.thickness)
            shroud_depth = (
                float(shroud.depth) if shroud.depth is not None else max(thickness * 0.80, 1.0e-4)
            )
            shroud_clearance = float(shroud.clearance)
            shroud_lip_depth = float(shroud.lip_depth)
            if shroud_thickness <= 0.0:
                raise ValueError("shroud.thickness must be positive")
            if shroud_depth <= 0.0:
                raise ValueError("shroud.depth must be positive")
            if shroud_clearance < 0.0:
                raise ValueError("shroud.clearance must be non-negative")
            if shroud_lip_depth < 0.0:
                raise ValueError("shroud.lip_depth must be non-negative")
            if tip_clearance > 1.0e-9:
                raise ValueError("blade.tip_clearance cannot be used with shroud")
            ring_outer_radius = outer_radius - shroud_clearance
            ring_inner_radius = ring_outer_radius - shroud_thickness
            if ring_outer_radius <= 0.0 or ring_inner_radius <= 0.0:
                raise ValueError("shroud dimensions exceed the rotor envelope")
            if ring_inner_radius <= hub_radius + max(thickness * 0.25, 1.0e-4):
                raise ValueError("shroud leaves no usable blade span")

        def lerp(a: float, b: float, t: float) -> float:
            return float(a) + (float(b) - float(a)) * float(t)

        def chord_scale(t: float) -> float:
            if blade_shape == "straight":
                return 1.00
            if blade_shape == "scimitar":
                return 1.08 - 0.16 * t + 0.05 * sin(pi * t)
            if blade_shape == "broad":
                return 1.05 + 0.10 * sin(pi * t) - 0.03 * t
            return 0.92 - 0.12 * t

        def sweep_factor(t: float) -> float:
            if blade_shape == "straight":
                return t
            if blade_shape == "scimitar":
                return min(1.25, 0.52 * t + 0.76 * t * t)
            if blade_shape == "broad":
                return min(1.08, 0.82 * t + 0.22 * sin(pi * t))
            return 0.82 * t

        cq = require_cadquery(feature="FanRotorGeometry")

        def rotate_section_points(
            points: list[tuple[float, float]],
            angle_rad: float,
        ) -> list[tuple[float, float]]:
            c = cos(angle_rad)
            s = sin(angle_rad)
            return [(y * c - z * s, y * s + z * c) for (y, z) in points]

        def blade_section_points(
            chord: float,
            section_thickness: float,
            pitch_deg: float,
            sweep_y: float,
            span_t: float,
        ) -> list[tuple[float, float]]:
            half_t = max(section_thickness * 0.5, chord * 0.011)
            camber_amplitude = camber * chord * 0.060
            skew_gain = 0.0
            if blade_shape == "scimitar":
                skew_gain = chord * (0.05 + 0.08 * span_t)
            elif blade_shape == "broad":
                skew_gain = chord * 0.03 * span_t
            elif blade_shape == "narrow":
                skew_gain = -chord * 0.04 * span_t

            upper: list[tuple[float, float]] = []
            lower: list[tuple[float, float]] = []
            for u in (0.00, 0.10, 0.24, 0.42, 0.62, 0.82, 1.00):
                chord_pos = (u - 0.5) * chord
                thickness_scale = max(0.12, sin(pi * u) ** 0.82)
                thickness_z = half_t * thickness_scale
                camber_z = camber_amplitude * sin(pi * u)
                skew_y = skew_gain * sin(pi * u) * (u - 0.20)
                upper.append((chord_pos + skew_y, camber_z + thickness_z))
                lower.append((chord_pos + skew_y, camber_z - thickness_z * 0.90))

            raw = upper + list(reversed(lower[1:-1]))
            rotated = rotate_section_points(raw, pitch_deg * pi / 180.0)
            return [(sweep_y + y, z) for (y, z) in rotated]

        hub_body_height = thickness * 0.36
        rear_collar_height = (
            float(hub.rear_collar_height)
            if hub.rear_collar_height is not None
            else thickness * 0.18
        )
        rear_collar_radius = (
            float(hub.rear_collar_radius)
            if hub.rear_collar_radius is not None
            else hub_radius * 0.78
        )
        if rear_collar_height < 0.0:
            raise ValueError("hub.rear_collar_height must be non-negative")
        if rear_collar_height > 1.0e-9 and rear_collar_radius <= 0.0:
            raise ValueError("hub.rear_collar_radius must be positive when rear_collar_height > 0")
        if rear_collar_radius >= hub_radius * 1.05:
            raise ValueError("hub.rear_collar_radius must fit within the hub radius")

        hub_body = cq.Workplane("XY").circle(hub_radius).extrude(hub_body_height * 0.5, both=True)
        shape = hub_body
        if rear_collar_height > 1.0e-9:
            rear_collar = (
                cq.Workplane("XY")
                .circle(rear_collar_radius)
                .extrude(rear_collar_height * 0.5, both=True)
                .translate((0.0, 0.0, -thickness * 0.24))
            )
            shape = shape.union(rear_collar)

        if hub_style == "capped":
            front_cap = (
                cq.Workplane("XY")
                .circle(hub_radius * 0.82)
                .extrude(max(thickness * 0.18, 1.0e-4))
                .translate((0.0, 0.0, hub_body_height * 0.5))
            )
            shape = shape.union(front_cap)
        elif hub_style in {"domed", "spinner"}:
            front_cap_height = thickness * (0.34 if hub_style == "domed" else 0.52)
            front_top_radius = hub_radius * (0.40 if hub_style == "domed" else 0.05)
            front_cap = (
                cq.Workplane("XY")
                .workplane(offset=hub_body_height * 0.5)
                .circle(hub_radius * 0.92)
                .workplane(offset=front_cap_height)
                .circle(max(front_top_radius, 1.0e-4))
                .loft(combine=True, ruled=False)
            )
            shape = shape.union(front_cap)

        if hub.bore_diameter is not None:
            bore_diameter = float(hub.bore_diameter)
            if bore_diameter <= 0.0:
                raise ValueError("hub.bore_diameter must be positive")
            bore_radius = bore_diameter * 0.5
            bore_limit = min(
                hub_radius, rear_collar_radius if rear_collar_height > 0.0 else hub_radius
            )
            if bore_radius >= bore_limit * 0.92:
                raise ValueError("hub.bore_diameter is too large for the hub")
            bore = (
                cq.Workplane("XY")
                .circle(bore_radius)
                .extrude(
                    max(thickness * 1.8, hub_body_height + rear_collar_height + shroud_depth),
                    both=True,
                )
            )
            shape = shape.cut(bore)

        root_radius = hub_radius * 0.82
        if shroud is not None:
            assert ring_inner_radius is not None
            tip_bridge = min(shroud_thickness * 0.18, max(thickness * 0.05, 5.0e-4))
            tip_radius = ring_inner_radius + tip_bridge
        else:
            tip_radius = outer_radius - tip_clearance
        if tip_radius <= root_radius + max(thickness * 0.22, 1.0e-4):
            raise ValueError("blade.tip_clearance leaves no usable blade span")

        blade_span = tip_radius - root_radius
        station_fracs = (0.00, 0.18, 0.42, 0.70, 1.00)
        stations = [root_radius + blade_span * frac for frac in station_fracs]
        section_chords = [
            lerp(root_chord, tip_chord, frac) * chord_scale(frac) for frac in station_fracs
        ]
        root_pitch_deg = blade_pitch_deg * 1.16
        section_pitches = [
            lerp(root_pitch_deg, tip_pitch_deg, frac**0.82) for frac in station_fracs
        ]
        root_section_t = max(thickness * 0.18, root_chord * 0.080)
        tip_section_t = max(thickness * 0.08, tip_chord * 0.048)
        thickness_shape_scale = {
            "straight": 1.00,
            "scimitar": 0.94,
            "broad": 1.08,
            "narrow": 0.88,
        }[blade_shape]
        section_thicknesses = [
            lerp(root_section_t, tip_section_t, frac**0.78) * thickness_shape_scale
            for frac in station_fracs
        ]
        sweep_amount = blade_span * sin(blade_sweep_deg * pi / 180.0) * 0.34
        section_sweeps = [sweep_amount * sweep_factor(frac) for frac in station_fracs]

        blade_wp = None
        previous_station = 0.0
        for station, chord, section_t, pitch_deg, sweep_y, span_t in zip(
            stations,
            section_chords,
            section_thicknesses,
            section_pitches,
            section_sweeps,
            station_fracs,
        ):
            section = blade_section_points(chord, section_t, pitch_deg, sweep_y, span_t)
            if blade_wp is None:
                blade_wp = cq.Workplane("YZ").workplane(offset=station).polyline(section).close()
            else:
                blade_wp = (
                    blade_wp.workplane(offset=station - previous_station).polyline(section).close()
                )
            previous_station = station
        assert blade_wp is not None
        blade_solid = blade_wp.loft(combine=True, ruled=False)

        angle_step = 360.0 / float(blade_count)
        for blade_index in range(blade_count):
            shape = shape.union(
                blade_solid.rotate(
                    (0.0, 0.0, 0.0),
                    (0.0, 0.0, 1.0),
                    angle_step * float(blade_index),
                )
            )

        if shroud is not None:
            assert ring_outer_radius is not None
            assert ring_inner_radius is not None
            tip_ring = (
                cq.Workplane("XY")
                .circle(ring_outer_radius)
                .circle(ring_inner_radius)
                .extrude(shroud_depth * 0.5, both=True)
            )
            shape = shape.union(tip_ring)
            if shroud_lip_depth > 1.0e-9:
                front_lip = (
                    cq.Workplane("XY")
                    .workplane(offset=shroud_depth * 0.5)
                    .circle(ring_outer_radius)
                    .circle(ring_inner_radius)
                    .extrude(shroud_lip_depth)
                )
                shape = shape.union(front_lip)

        geom = _mesh_geometry_from_cadquery_model(shape)
        if not center:
            geom = _mesh_geometry_shifted_to_z0(geom)
        _adopt_mesh_geometry(self, geom)


class BlowerWheelGeometry(MeshGeometry):
    """
    Build a squirrel-cage blower wheel around a central drum.
    """

    def __init__(
        self,
        outer_radius: float,
        inner_radius: float,
        width: float,
        blade_count: int,
        *,
        blade_thickness: float,
        blade_sweep_deg: float = 35.0,
        backplate: bool = True,
        shroud: bool = True,
        center: bool = True,
    ):
        super().__init__()
        outer_radius = float(outer_radius)
        inner_radius = float(inner_radius)
        width = float(width)
        blade_count = int(blade_count)
        blade_thickness = float(blade_thickness)
        blade_sweep_deg = float(blade_sweep_deg)

        if outer_radius <= 0.0 or inner_radius <= 0.0 or width <= 0.0:
            raise ValueError("outer_radius, inner_radius, and width must be positive")
        if inner_radius >= outer_radius:
            raise ValueError("inner_radius must be less than outer_radius")
        if blade_count < 2:
            raise ValueError("blade_count must be at least 2")
        if blade_thickness <= 0.0:
            raise ValueError("blade_thickness must be positive")
        if abs(blade_sweep_deg) >= 85.0:
            raise ValueError("abs(blade_sweep_deg) must be < 85")

        radial_span = outer_radius - inner_radius
        if blade_thickness >= radial_span * 0.9:
            raise ValueError("blade_thickness is too large for the blower annulus")

        cq = require_cadquery(feature="BlowerWheelGeometry")
        drum_wall = min(max(blade_thickness * 1.8, radial_span * 0.10), inner_radius * 0.55)
        if drum_wall <= 1e-6 or inner_radius - drum_wall <= 1e-6:
            raise ValueError("inner_radius is too small for the blower drum wall")
        drum = (
            cq.Workplane("XY")
            .circle(inner_radius)
            .circle(inner_radius - drum_wall)
            .extrude(width * 0.5, both=True)
        )

        shape = drum
        side_plate_thickness = min(max(blade_thickness * 1.1, width * 0.06), width * 0.16)
        side_plate_inner_radius = max(inner_radius - drum_wall * 0.40, 1.0e-4)
        if backplate:
            rear_plate = (
                cq.Workplane("XY")
                .circle(outer_radius)
                .circle(side_plate_inner_radius)
                .extrude(side_plate_thickness, both=False)
                .translate((0.0, 0.0, -width * 0.5))
            )
            shape = shape.union(rear_plate)
        if shroud:
            front_plate = (
                cq.Workplane("XY")
                .circle(outer_radius)
                .circle(side_plate_inner_radius)
                .extrude(side_plate_thickness, both=False)
                .translate((0.0, 0.0, width * 0.5 - side_plate_thickness))
            )
            shape = shape.union(front_plate)

        sweep_rad = blade_sweep_deg * pi / 180.0
        inner_attach_radius = max(inner_radius - drum_wall * 0.08, 1.0e-4)
        outer_attach_radius = outer_radius - blade_thickness * 0.35
        rear_clear = side_plate_thickness * 0.95 if backplate else blade_thickness * 0.50
        front_clear = side_plate_thickness * 0.95 if shroud else blade_thickness * 0.50
        blade_length = width - rear_clear - front_clear
        if blade_length <= blade_thickness * 1.5:
            raise ValueError("width is too small for the blower blade depth")
        blade_center_z = (rear_clear - front_clear) * 0.5

        def annulus_point(radius: float, angle_rad: float) -> tuple[float, float]:
            return (radius * cos(angle_rad), radius * sin(angle_rad))

        def thicken_centerline(
            centerline: list[tuple[float, float]],
            strip_thickness: float,
        ) -> list[tuple[float, float]]:
            half_t = strip_thickness * 0.5
            positive: list[tuple[float, float]] = []
            negative: list[tuple[float, float]] = []
            count = len(centerline)
            for index, point in enumerate(centerline):
                if index == 0:
                    tangent = (
                        centerline[1][0] - point[0],
                        centerline[1][1] - point[1],
                    )
                elif index == count - 1:
                    tangent = (
                        point[0] - centerline[index - 1][0],
                        point[1] - centerline[index - 1][1],
                    )
                else:
                    tangent = (
                        centerline[index + 1][0] - centerline[index - 1][0],
                        centerline[index + 1][1] - centerline[index - 1][1],
                    )
                tangent_norm = sqrt(tangent[0] * tangent[0] + tangent[1] * tangent[1])
                if tangent_norm <= _EPS:
                    normal = (0.0, 1.0)
                else:
                    normal = (-tangent[1] / tangent_norm, tangent[0] / tangent_norm)
                positive.append((point[0] + normal[0] * half_t, point[1] + normal[1] * half_t))
                negative.append((point[0] - normal[0] * half_t, point[1] - normal[1] * half_t))
            return positive + list(reversed(negative))

        def annular_sector_polygon(
            inner_r: float,
            outer_r: float,
            start_angle: float,
            end_angle: float,
            *,
            segments: int = 8,
        ) -> list[tuple[float, float]]:
            outer_points = [
                annulus_point(
                    outer_r,
                    start_angle + (end_angle - start_angle) * (index / float(segments)),
                )
                for index in range(segments + 1)
            ]
            inner_points = [
                annulus_point(
                    inner_r,
                    end_angle - (end_angle - start_angle) * (index / float(segments)),
                )
                for index in range(segments + 1)
            ]
            return outer_points + inner_points

        angle_step = 2.0 * pi / float(blade_count)
        drum_window_inner_radius = max(inner_radius - drum_wall - 1.0e-4, 1.0e-4)
        drum_window_outer_radius = inner_radius + 1.0e-4
        drum_window_span = angle_step * 0.34
        drum_window_length = width - 2.0 * max(side_plate_thickness * 1.15, blade_thickness * 0.8)
        if drum_window_length > blade_thickness:
            for window_index in range(blade_count):
                passage_center = window_index * angle_step + angle_step * 0.5 - sweep_rad * 0.18
                window_profile = annular_sector_polygon(
                    drum_window_inner_radius,
                    drum_window_outer_radius,
                    passage_center - drum_window_span * 0.5,
                    passage_center + drum_window_span * 0.5,
                )
                drum_window = (
                    cq.Workplane("XY")
                    .polyline(window_profile)
                    .close()
                    .extrude(drum_window_length * 0.5, both=True)
                )
                shape = shape.cut(drum_window)

        for blade_index in range(blade_count):
            base_angle = blade_index * angle_step
            centerline = [
                annulus_point(inner_attach_radius, base_angle - sweep_rad * 0.18),
                annulus_point(
                    inner_attach_radius + radial_span * 0.34, base_angle + sweep_rad * 0.10
                ),
                annulus_point(
                    inner_attach_radius + radial_span * 0.68, base_angle + sweep_rad * 0.42
                ),
                annulus_point(outer_attach_radius, base_angle + sweep_rad * 0.72),
            ]
            blade_profile = thicken_centerline(centerline, blade_thickness)
            blade = (
                cq.Workplane("XY")
                .polyline(blade_profile)
                .close()
                .extrude(blade_length * 0.5, both=True)
                .translate((0.0, 0.0, blade_center_z))
            )
            shape = shape.union(blade)

        geom = _mesh_geometry_from_cadquery_model(shape)
        if not center:
            geom = _mesh_geometry_shifted_to_z0(geom)
        _adopt_mesh_geometry(self, geom)
