from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


MAIN_CANOPY_RADIUS = 0.080
MAIN_CANOPY_INNER_RADIUS = 0.072
MAIN_CANOPY_HEIGHT = 0.120
CANOPY_CAP_THICKNESS = 0.006
UPPER_STEM_OUTER_RADIUS = 0.024
UPPER_STEM_INNER_RADIUS = 0.018
UPPER_STEM_HEIGHT = 0.085
OUTLET_OUTER_RADIUS = 0.0135
OUTLET_INNER_RADIUS = 0.0045
OUTLET_HEIGHT = 0.014
OUTLET_Z = -MAIN_CANOPY_HEIGHT

DROP_TRAVEL = 0.20
CORD_RADIUS = 0.0035
CORD_TOP = DROP_TRAVEL + 0.02
CORD_END = -0.190
CORD_LENGTH = CORD_TOP - CORD_END
CORD_CENTER_Z = (CORD_TOP + CORD_END) * 0.5
FERRULE_RADIUS = 0.009
FERRULE_LENGTH = 0.016
FERRULE_CENTER_Z = -(OUTLET_HEIGHT + (FERRULE_LENGTH * 0.5))
SWIVEL_RADIUS = 0.011
SWIVEL_LENGTH = 0.020
SWIVEL_CENTER_Z = CORD_END - (SWIVEL_LENGTH * 0.5)
SWIVEL_BOTTOM_Z = CORD_END - SWIVEL_LENGTH

SHADE_OUTER_RADIUS = 0.130
SHADE_INNER_RADIUS = 0.122
SHADE_HEIGHT = 0.180
SHADE_TOP_CAP_THICKNESS = 0.008
SHADE_HUB_RADIUS = 0.016
SHADE_HUB_LENGTH = 0.032
SHADE_HUB_CENTER_Z = -SHADE_HUB_LENGTH * 0.5


def _circle_profile(radius: float, segments: int = 48) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * i) / segments),
            radius * math.sin((2.0 * math.pi * i) / segments),
        )
        for i in range(segments)
    ]


def _ring_mesh(
    *,
    name: str,
    outer_radius: float,
    inner_radius: float,
    height: float,
):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(outer_radius),
            [_circle_profile(inner_radius)],
            height=height,
            center=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retractable_pendant_light")

    canopy_paint = model.material("canopy_paint", rgba=(0.16, 0.17, 0.18, 1.0))
    shade_paint = model.material("shade_paint", rgba=(0.20, 0.21, 0.22, 1.0))
    mechanism_metal = model.material("mechanism_metal", rgba=(0.70, 0.72, 0.74, 1.0))
    cord_black = model.material("cord_black", rgba=(0.06, 0.06, 0.07, 1.0))

    canopy_wall_mesh = _ring_mesh(
        name="canopy_wall",
        outer_radius=MAIN_CANOPY_RADIUS,
        inner_radius=MAIN_CANOPY_INNER_RADIUS,
        height=MAIN_CANOPY_HEIGHT,
    )
    canopy_top_mesh = _ring_mesh(
        name="canopy_top",
        outer_radius=MAIN_CANOPY_RADIUS,
        inner_radius=UPPER_STEM_INNER_RADIUS,
        height=CANOPY_CAP_THICKNESS,
    )
    canopy_bottom_mesh = _ring_mesh(
        name="canopy_bottom",
        outer_radius=MAIN_CANOPY_RADIUS,
        inner_radius=OUTLET_OUTER_RADIUS,
        height=CANOPY_CAP_THICKNESS,
    )
    stem_wall_mesh = _ring_mesh(
        name="ceiling_stem",
        outer_radius=UPPER_STEM_OUTER_RADIUS,
        inner_radius=UPPER_STEM_INNER_RADIUS,
        height=UPPER_STEM_HEIGHT,
    )
    stem_top_mesh = _ring_mesh(
        name="ceiling_stem_top",
        outer_radius=UPPER_STEM_OUTER_RADIUS,
        inner_radius=0.011,
        height=CANOPY_CAP_THICKNESS,
    )
    outlet_collar_mesh = _ring_mesh(
        name="outlet_collar",
        outer_radius=OUTLET_OUTER_RADIUS,
        inner_radius=OUTLET_INNER_RADIUS,
        height=OUTLET_HEIGHT,
    )

    canopy = model.part("canopy")
    canopy.visual(
        canopy_wall_mesh,
        origin=Origin(xyz=(0.0, 0.0, -MAIN_CANOPY_HEIGHT * 0.5)),
        material=canopy_paint,
        name="canopy_wall",
    )
    canopy.visual(
        canopy_top_mesh,
        origin=Origin(xyz=(0.0, 0.0, -CANOPY_CAP_THICKNESS * 0.5)),
        material=canopy_paint,
        name="canopy_top",
    )
    canopy.visual(
        canopy_bottom_mesh,
        origin=Origin(
            xyz=(0.0, 0.0, -MAIN_CANOPY_HEIGHT + (CANOPY_CAP_THICKNESS * 0.5))
        ),
        material=canopy_paint,
        name="canopy_bottom",
    )
    canopy.visual(
        stem_wall_mesh,
        origin=Origin(xyz=(0.0, 0.0, UPPER_STEM_HEIGHT * 0.5)),
        material=canopy_paint,
        name="ceiling_stem",
    )
    canopy.visual(
        stem_top_mesh,
        origin=Origin(
            xyz=(0.0, 0.0, UPPER_STEM_HEIGHT - (CANOPY_CAP_THICKNESS * 0.5))
        ),
        material=canopy_paint,
        name="ceiling_stem_top",
    )
    canopy.visual(
        outlet_collar_mesh,
        origin=Origin(
            xyz=(0.0, 0.0, OUTLET_Z - (OUTLET_HEIGHT * 0.5))
        ),
        material=mechanism_metal,
        name="outlet_collar",
    )
    canopy.inertial = Inertial.from_geometry(
        Box((0.17, 0.17, 0.22)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, -0.02)),
    )

    drop_head = model.part("drop_head")
    drop_head.visual(
        Cylinder(radius=CORD_RADIUS, length=CORD_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, CORD_CENTER_Z)),
        material=cord_black,
        name="cord",
    )
    drop_head.visual(
        Cylinder(radius=FERRULE_RADIUS, length=FERRULE_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, FERRULE_CENTER_Z)),
        material=mechanism_metal,
        name="seat_ferrule",
    )
    drop_head.visual(
        Cylinder(radius=SWIVEL_RADIUS, length=SWIVEL_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, SWIVEL_CENTER_Z)),
        material=mechanism_metal,
        name="swivel_barrel",
    )
    drop_head.inertial = Inertial.from_geometry(
        Box((0.03, 0.03, CORD_LENGTH + 0.02)),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.0, CORD_CENTER_Z)),
    )

    shade_wall_mesh = _ring_mesh(
        name="shade_wall",
        outer_radius=SHADE_OUTER_RADIUS,
        inner_radius=SHADE_INNER_RADIUS,
        height=SHADE_HEIGHT,
    )
    shade_top_mesh = _ring_mesh(
        name="shade_top",
        outer_radius=0.126,
        inner_radius=0.014,
        height=SHADE_TOP_CAP_THICKNESS,
    )
    shade_lip_mesh = _ring_mesh(
        name="shade_lip",
        outer_radius=0.132,
        inner_radius=0.124,
        height=0.008,
    )

    shade = model.part("shade")
    shade.visual(
        Cylinder(radius=SHADE_HUB_RADIUS, length=SHADE_HUB_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, SHADE_HUB_CENTER_Z)),
        material=mechanism_metal,
        name="shade_hub",
    )
    shade.visual(
        shade_wall_mesh,
        origin=Origin(xyz=(0.0, 0.0, -SHADE_HEIGHT * 0.5)),
        material=shade_paint,
        name="shade_wall",
    )
    shade.visual(
        shade_top_mesh,
        origin=Origin(xyz=(0.0, 0.0, -SHADE_TOP_CAP_THICKNESS * 0.5)),
        material=shade_paint,
        name="shade_top",
    )
    shade.visual(
        shade_lip_mesh,
        origin=Origin(xyz=(0.0, 0.0, -SHADE_HEIGHT + 0.004)),
        material=shade_paint,
        name="shade_lip",
    )
    shade.inertial = Inertial.from_geometry(
        Cylinder(radius=SHADE_OUTER_RADIUS, length=SHADE_HEIGHT),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, -SHADE_HEIGHT * 0.5)),
    )

    model.articulation(
        "canopy_to_drop",
        ArticulationType.PRISMATIC,
        parent=canopy,
        child=drop_head,
        origin=Origin(xyz=(0.0, 0.0, OUTLET_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=0.35,
            lower=0.0,
            upper=DROP_TRAVEL,
        ),
    )
    model.articulation(
        "drop_to_shade_spin",
        ArticulationType.CONTINUOUS,
        parent=drop_head,
        child=shade,
        origin=Origin(xyz=(0.0, 0.0, SWIVEL_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    canopy = object_model.get_part("canopy")
    drop_head = object_model.get_part("drop_head")
    shade = object_model.get_part("shade")
    drop_joint = object_model.get_articulation("canopy_to_drop")
    spin_joint = object_model.get_articulation("drop_to_shade_spin")

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

    ctx.check(
        "drop mechanism is prismatic",
        drop_joint.articulation_type == ArticulationType.PRISMATIC,
        details=f"joint_type={drop_joint.articulation_type}",
    )
    ctx.check(
        "shade spinner is continuous",
        spin_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint_type={spin_joint.articulation_type}",
    )
    ctx.check(
        "drop axis points downward",
        tuple(round(v, 6) for v in drop_joint.axis) == (0.0, 0.0, -1.0),
        details=f"axis={drop_joint.axis}",
    )
    ctx.check(
        "shade spin axis is vertical",
        tuple(round(v, 6) for v in spin_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={spin_joint.axis}",
    )

    ctx.expect_contact(
        drop_head,
        canopy,
        elem_a="seat_ferrule",
        elem_b="outlet_collar",
        name="retracted ferrule seats in outlet collar",
    )
    ctx.expect_contact(
        drop_head,
        shade,
        elem_a="swivel_barrel",
        elem_b="shade_hub",
        name="shade hangs from swivel barrel",
    )
    ctx.expect_gap(
        canopy,
        shade,
        axis="z",
        min_gap=0.05,
        name="shade clears canopy in the retracted pose",
    )
    ctx.expect_overlap(
        canopy,
        shade,
        axes="xy",
        min_overlap=0.12,
        name="shade stays centered beneath canopy",
    )

    rest_pos = ctx.part_world_position(shade)
    drop_upper = drop_joint.motion_limits.upper if drop_joint.motion_limits else None
    if drop_upper is None:
        ctx.fail("drop travel limit missing", "Prismatic drop joint must define an upper limit.")
        return ctx.report()

    with ctx.pose({drop_joint: drop_upper}):
        extended_pos = ctx.part_world_position(shade)
        ctx.expect_overlap(
            drop_head,
            canopy,
            axes="z",
            elem_a="cord",
            elem_b="outlet_collar",
            min_overlap=0.010,
            name="cord remains captured in the outlet collar at full drop",
        )
        ctx.expect_overlap(
            drop_head,
            canopy,
            axes="xy",
            elem_a="cord",
            elem_b="outlet_collar",
            min_overlap=0.006,
            name="cord stays aligned through the outlet collar",
        )
        ctx.expect_gap(
            canopy,
            shade,
            axis="z",
            min_gap=0.24,
            name="shade hangs noticeably lower when pulled down",
        )

    ctx.check(
        "shade lowers when the drop joint extends",
        rest_pos is not None and extended_pos is not None and extended_pos[2] < rest_pos[2] - 0.15,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
