from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from math import cos, sin, tau

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_FOOT_RADIUS = 0.097
BASE_BODY_RADIUS = 0.084
BASE_HEIGHT = 0.100
BASE_FOOT_THICKNESS = 0.012
BASE_FLOOR_THICKNESS = 0.024
BASE_TOP_RING_THICKNESS = 0.012
BASE_CAVITY_RADIUS = 0.066
BASE_BORE_RADIUS = 0.024
BASE_SLEEVE_OUTER_RADIUS = 0.040
BASE_SLEEVE_TOP_Z = 0.074
BASE_WALL_HEIGHT = BASE_HEIGHT - BASE_FOOT_THICKNESS - BASE_TOP_RING_THICKNESS

SPINDLE_BOTTOM_Z = -0.072
SPINDLE_JOURNAL_RADIUS = 0.022
RETAINER_RADIUS = 0.050
RETAINER_Z0 = -0.020
RETAINER_HEIGHT = 0.006
THRUST_COLLAR_RADIUS = 0.046
THRUST_COLLAR_HEIGHT = 0.007
UPPER_SPINDLE_RADIUS = 0.026
UPPER_SPINDLE_Z1 = 0.030
HUB_RADIUS = 0.060
HUB_Z0 = 0.030
HUB_Z1 = 0.050
STIFFENER_RING_INNER_RADIUS = 0.110
STIFFENER_RING_OUTER_RADIUS = 0.145
STIFFENER_RING_Z0 = 0.042
STIFFENER_RING_HEIGHT = 0.008
RIB_COUNT = 6
RIB_WIDTH = 0.016
RIB_HEIGHT = 0.010
RIB_CENTER_RADIUS = 0.086
RIB_LENGTH = 0.058
RIB_Z0 = 0.040
TOP_PLATE_RADIUS = 0.170
TOP_PLATE_Z0 = 0.050
TOP_PLATE_THICKNESS = 0.016
PLATEN_TOP_Z = TOP_PLATE_Z0 + TOP_PLATE_THICKNESS


def _cylinder(radius: float, height: float, z0: float = 0.0) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(height).translate((0.0, 0.0, z0))


def _ring(inner_radius: float, outer_radius: float, height: float, z0: float = 0.0) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
        .translate((0.0, 0.0, z0))
    )


def _aabb_size(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
    return tuple(aabb[1][axis] - aabb[0][axis] for axis in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vertical_labeling_platen")

    model.material("housing_charcoal", rgba=(0.19, 0.20, 0.22, 1.0))
    model.material("machined_aluminum", rgba=(0.77, 0.78, 0.80, 1.0))

    base_housing = model.part("base_housing")
    base_housing.visual(
        Cylinder(radius=BASE_FOOT_RADIUS, length=BASE_FOOT_THICKNESS),
        material="housing_charcoal",
        origin=Origin(xyz=(0.0, 0.0, BASE_FOOT_THICKNESS / 2.0)),
        name="foot",
    )
    base_housing.visual(
        mesh_from_cadquery(
            _ring(
                BASE_BORE_RADIUS,
                BASE_CAVITY_RADIUS,
                BASE_FLOOR_THICKNESS - BASE_FOOT_THICKNESS,
                BASE_FOOT_THICKNESS,
            ),
            "base_floor_bridge",
        ),
        material="housing_charcoal",
        name="floor_bridge",
    )
    base_housing.visual(
        mesh_from_cadquery(
            _ring(BASE_CAVITY_RADIUS, BASE_BODY_RADIUS, BASE_WALL_HEIGHT, BASE_FOOT_THICKNESS),
            "base_outer_wall",
        ),
        material="housing_charcoal",
        name="outer_wall",
    )
    base_housing.visual(
        mesh_from_cadquery(
            _ring(BASE_SLEEVE_OUTER_RADIUS, BASE_BODY_RADIUS, BASE_TOP_RING_THICKNESS, BASE_HEIGHT - BASE_TOP_RING_THICKNESS),
            "base_top_cover",
        ),
        material="housing_charcoal",
        name="top_cover",
    )
    base_housing.visual(
        mesh_from_cadquery(
            _ring(BASE_BORE_RADIUS, BASE_SLEEVE_OUTER_RADIUS, BASE_SLEEVE_TOP_Z - BASE_FLOOR_THICKNESS, BASE_FLOOR_THICKNESS),
            "bearing_sleeve",
        ),
        material="housing_charcoal",
        name="bearing_sleeve",
    )
    base_housing.inertial = Inertial.from_geometry(
        Cylinder(radius=BASE_FOOT_RADIUS, length=BASE_HEIGHT),
        mass=8.2,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT / 2.0)),
    )

    rotating_platen = model.part("rotating_platen")
    rotating_platen.visual(
        Cylinder(radius=SPINDLE_JOURNAL_RADIUS, length=HUB_Z0 - SPINDLE_BOTTOM_Z),
        material="machined_aluminum",
        origin=Origin(xyz=(0.0, 0.0, (HUB_Z0 + SPINDLE_BOTTOM_Z) / 2.0)),
        name="journal",
    )
    rotating_platen.visual(
        Cylinder(radius=RETAINER_RADIUS, length=RETAINER_HEIGHT),
        material="machined_aluminum",
        origin=Origin(xyz=(0.0, 0.0, RETAINER_Z0 + (RETAINER_HEIGHT / 2.0))),
        name="retainer_flange",
    )
    rotating_platen.visual(
        Cylinder(radius=THRUST_COLLAR_RADIUS, length=THRUST_COLLAR_HEIGHT),
        material="machined_aluminum",
        origin=Origin(xyz=(0.0, 0.0, THRUST_COLLAR_HEIGHT / 2.0)),
        name="thrust_collar",
    )
    rotating_platen.visual(
        Cylinder(radius=UPPER_SPINDLE_RADIUS, length=UPPER_SPINDLE_Z1 - THRUST_COLLAR_HEIGHT),
        material="machined_aluminum",
        origin=Origin(xyz=(0.0, 0.0, (UPPER_SPINDLE_Z1 + THRUST_COLLAR_HEIGHT) / 2.0)),
        name="upper_spindle",
    )
    rotating_platen.visual(
        Cylinder(radius=HUB_RADIUS, length=HUB_Z1 - HUB_Z0),
        material="machined_aluminum",
        origin=Origin(xyz=(0.0, 0.0, (HUB_Z0 + HUB_Z1) / 2.0)),
        name="hub",
    )
    rotating_platen.visual(
        mesh_from_cadquery(
            _ring(STIFFENER_RING_INNER_RADIUS, STIFFENER_RING_OUTER_RADIUS, STIFFENER_RING_HEIGHT, STIFFENER_RING_Z0),
            "stiffener_ring",
        ),
        material="machined_aluminum",
        name="stiffener_ring",
    )
    rotating_platen.visual(
        Cylinder(radius=TOP_PLATE_RADIUS, length=TOP_PLATE_THICKNESS),
        material="machined_aluminum",
        origin=Origin(xyz=(0.0, 0.0, TOP_PLATE_Z0 + (TOP_PLATE_THICKNESS / 2.0))),
        name="top_plate",
    )
    for rib_index in range(RIB_COUNT):
        angle = rib_index * tau / RIB_COUNT
        rotating_platen.visual(
            Box((RIB_LENGTH, RIB_WIDTH, RIB_HEIGHT)),
            material="machined_aluminum",
            origin=Origin(
                xyz=(RIB_CENTER_RADIUS * cos(angle), RIB_CENTER_RADIUS * sin(angle), RIB_Z0 + (RIB_HEIGHT / 2.0)),
                rpy=(0.0, 0.0, angle),
            ),
            name=f"rib_{rib_index + 1}",
        )

    rotating_platen.inertial = Inertial.from_geometry(
        Cylinder(radius=TOP_PLATE_RADIUS, length=PLATEN_TOP_Z - SPINDLE_BOTTOM_Z),
        mass=4.3,
        origin=Origin(xyz=(0.0, 0.0, (PLATEN_TOP_Z + SPINDLE_BOTTOM_Z) / 2.0)),
    )

    model.articulation(
        "platen_spin",
        ArticulationType.CONTINUOUS,
        parent=base_housing,
        child=rotating_platen,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_housing = object_model.get_part("base_housing")
    rotating_platen = object_model.get_part("rotating_platen")
    platen_spin = object_model.get_articulation("platen_spin")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        rotating_platen,
        base_housing,
        elem_a="thrust_collar",
        elem_b="top_cover",
        name="thrust collar seats on the housing cover",
    )
    ctx.expect_overlap(
        rotating_platen,
        base_housing,
        axes="xy",
        min_overlap=0.160,
        name="platen stays centered over the housing footprint",
    )
    ctx.expect_gap(
        rotating_platen,
        base_housing,
        axis="z",
        positive_elem="top_plate",
        negative_elem="top_cover",
        min_gap=0.045,
        name="broad work plate clears the housing with visible height",
    )
    ctx.expect_within(
        rotating_platen,
        base_housing,
        axes="xy",
        inner_elem="journal",
        outer_elem="bearing_sleeve",
        margin=0.0,
        name="journal stays centered within the bearing sleeve envelope",
    )

    with ctx.pose({platen_spin: 2.2}):
        ctx.expect_contact(
            rotating_platen,
            base_housing,
            elem_a="thrust_collar",
            elem_b="top_cover",
            name="spindle support stays seated after rotation",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="turned pose remains free of clashes")

    articulation_kind = getattr(platen_spin.articulation_type, "name", str(platen_spin.articulation_type))
    limits = platen_spin.motion_limits
    ctx.check(
        "spin articulation is continuous and vertical",
        articulation_kind == "CONTINUOUS"
        and tuple(round(value, 6) for value in platen_spin.axis) == (0.0, 0.0, 1.0)
        and limits is not None
        and limits.lower is None
        and limits.upper is None,
        details=(
            f"type={articulation_kind}, axis={platen_spin.axis}, "
            f"limits=({None if limits is None else limits.lower}, {None if limits is None else limits.upper})"
        ),
    )

    base_aabb = ctx.part_world_aabb(base_housing)
    platen_aabb = ctx.part_world_aabb(rotating_platen)
    if base_aabb is not None and platen_aabb is not None:
        base_size = _aabb_size(base_aabb)
        platen_size = _aabb_size(platen_aabb)
        spindle_bottom_z = platen_aabb[0][2]
        housing_top_z = base_aabb[1][2]
        housing_bottom_z = base_aabb[0][2]
        platen_top_z = platen_aabb[1][2]

        ctx.check(
            "top plate dominates the silhouette",
            platen_size[0] > (1.70 * base_size[0]) and platen_size[1] > (1.70 * base_size[1]),
            details=f"platen_xy={platen_size[:2]}, base_xy={base_size[:2]}",
        )
        ctx.check(
            "spindle sinks credibly into the housing",
            spindle_bottom_z < (housing_top_z - 0.050) and spindle_bottom_z > (housing_bottom_z + 0.020),
            details=(
                f"spindle_bottom_z={spindle_bottom_z:.4f}, "
                f"housing_z=({housing_bottom_z:.4f}, {housing_top_z:.4f})"
            ),
        )
        ctx.check(
            "platen rises clearly above the compact support",
            platen_top_z > (housing_top_z + 0.055),
            details=f"platen_top_z={platen_top_z:.4f}, housing_top_z={housing_top_z:.4f}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
