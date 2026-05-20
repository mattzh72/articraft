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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    wire_from_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="astronomy_binocular_on_yoke_mount")

    tripod_black = model.material("tripod_black", rgba=(0.13, 0.14, 0.15, 1.0))
    head_gray = model.material("head_gray", rgba=(0.28, 0.30, 0.33, 1.0))
    mount_gray = model.material("mount_gray", rgba=(0.41, 0.43, 0.46, 1.0))
    tube_white = model.material("tube_white", rgba=(0.90, 0.92, 0.94, 1.0))
    anodized_black = model.material("anodized_black", rgba=(0.10, 0.11, 0.12, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.34, 0.45, 0.54, 0.55))

    def straight_tube(name: str, start: tuple[float, float, float], end: tuple[float, float, float], radius: float):
        return mesh_from_geometry(
            wire_from_points(
                [start, end],
                radius=radius,
                cap_ends=True,
            ),
            name,
        )

    tripod = model.part("tripod_head")
    tripod.visual(
        Cylinder(radius=0.047, length=0.68),
        origin=Origin(xyz=(0.0, 0.0, 0.74)),
        material=tripod_black,
        name="center_column",
    )
    tripod.visual(
        Cylinder(radius=0.105, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.82)),
        material=head_gray,
        name="leg_collar",
    )
    tripod.visual(
        Cylinder(radius=0.125, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 1.05)),
        material=head_gray,
        name="tripod_bowl",
    )
    tripod.visual(
        Cylinder(radius=0.105, length=0.01),
        origin=Origin(xyz=(0.0, 0.0, 1.095)),
        material=anodized_black,
        name="bearing_top",
    )
    tripod.visual(
        Cylinder(radius=0.055, length=0.02),
        origin=Origin(xyz=(0.0, 0.0, 0.47)),
        material=head_gray,
        name="spreader_hub",
    )

    leg_anchor_radius = 0.09
    foot_radius = 0.60
    leg_anchor_z = 0.79
    foot_z = 0.03
    spreader_radius = 0.27
    spreader_z = 0.50
    for index in range(3):
        angle = index * (2.0 * math.pi / 3.0) + (math.pi / 6.0)
        anchor = (
            leg_anchor_radius * math.cos(angle),
            leg_anchor_radius * math.sin(angle),
            leg_anchor_z,
        )
        foot = (
            foot_radius * math.cos(angle),
            foot_radius * math.sin(angle),
            foot_z,
        )
        spreader_end = (
            spreader_radius * math.cos(angle),
            spreader_radius * math.sin(angle),
            spreader_z,
        )
        tripod.visual(
            straight_tube(f"tripod_leg_mesh_{index}", anchor, foot, 0.018),
            material=tripod_black,
            name=f"tripod_leg_{index}",
        )
        tripod.visual(
            Cylinder(radius=0.035, length=0.02),
            origin=Origin(xyz=(foot[0], foot[1], 0.01)),
            material=anodized_black,
            name=f"tripod_foot_{index}",
        )
        tripod.visual(
            straight_tube(
                f"spreader_arm_mesh_{index}",
                (0.0, 0.0, 0.47),
                spreader_end,
                0.008,
            ),
            material=head_gray,
            name=f"spreader_arm_{index}",
        )
    tripod.inertial = Inertial.from_geometry(
        Cylinder(radius=0.14, length=1.12),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, 0.56)),
    )

    yoke = model.part("yoke_mount")
    yoke.visual(
        Cylinder(radius=0.110, length=0.02),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        material=anodized_black,
        name="azimuth_turntable",
    )
    yoke.visual(
        Cylinder(radius=0.090, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=mount_gray,
        name="azimuth_hub",
    )
    yoke.visual(
        Box((0.18, 0.14, 0.10)),
        origin=Origin(xyz=(0.0, -0.035, 0.11)),
        material=mount_gray,
        name="arm_pedestal",
    )
    yoke.visual(
        Box((0.12, 0.12, 0.42)),
        origin=Origin(xyz=(0.0, -0.070, 0.31)),
        material=mount_gray,
        name="yoke_arm",
    )
    yoke.visual(
        Box((0.28, 0.10, 0.16)),
        origin=Origin(xyz=(-0.08, -0.045, 0.20), rpy=(0.0, -0.70, 0.0)),
        material=head_gray,
        name="rear_brace",
    )
    yoke.visual(
        Cylinder(radius=0.070, length=0.03),
        origin=Origin(xyz=(0.0, -0.015, 0.52), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=anodized_black,
        name="altitude_bearing_plate",
    )
    yoke.visual(
        Cylinder(radius=0.028, length=0.025),
        origin=Origin(xyz=(0.0, -0.040, 0.52), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=head_gray,
        name="altitude_tension_knob",
    )
    yoke.inertial = Inertial.from_geometry(
        Box((0.32, 0.18, 0.58)),
        mass=5.0,
        origin=Origin(xyz=(-0.02, 0.0, 0.29)),
    )

    binocular = model.part("binocular_assembly")
    binocular.visual(
        Cylinder(radius=0.065, length=0.03),
        origin=Origin(xyz=(0.0, 0.015, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=anodized_black,
        name="trunnion_plate",
    )
    binocular.visual(
        Cylinder(radius=0.028, length=0.56),
        origin=Origin(xyz=(0.0, 0.31, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=mount_gray,
        name="cradle_bar",
    )
    binocular.visual(
        Cylinder(radius=0.036, length=0.04),
        origin=Origin(xyz=(0.0, 0.59, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=anodized_black,
        name="bar_end_cap",
    )

    tube_centers_y = (0.18, 0.40)
    for index, tube_y in enumerate(tube_centers_y):
        tube_label = "inner" if index == 0 else "outer"
        binocular.visual(
            Box((0.10, 0.055, 0.15)),
            origin=Origin(xyz=(0.01, tube_y, 0.075)),
            material=mount_gray,
            name=f"{tube_label}_saddle_block",
        )
        binocular.visual(
            Cylinder(radius=0.070, length=0.035),
            origin=Origin(xyz=(0.02, tube_y, 0.12), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=anodized_black,
            name=f"{tube_label}_tube_clamp",
        )
        binocular.visual(
            Cylinder(radius=0.055, length=0.72),
            origin=Origin(xyz=(0.26, tube_y, 0.12), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=tube_white,
            name=f"{tube_label}_main_tube",
        )
        binocular.visual(
            Cylinder(radius=0.062, length=0.18),
            origin=Origin(xyz=(0.71, tube_y, 0.12), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=tube_white,
            name=f"{tube_label}_dew_shield",
        )
        binocular.visual(
            Cylinder(radius=0.066, length=0.04),
            origin=Origin(xyz=(0.82, tube_y, 0.12), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=anodized_black,
            name=f"{tube_label}_objective_cell",
        )
        binocular.visual(
            Cylinder(radius=0.048, length=0.01),
            origin=Origin(xyz=(0.845, tube_y, 0.12), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=lens_glass,
            name=f"{tube_label}_objective_glass",
        )
        binocular.visual(
            Cylinder(radius=0.033, length=0.16),
            origin=Origin(xyz=(-0.12, tube_y, 0.12), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=anodized_black,
            name=f"{tube_label}_focuser_drawtube",
        )
        binocular.visual(
            Box((0.08, 0.05, 0.05)),
            origin=Origin(xyz=(-0.20, tube_y, 0.145)),
            material=anodized_black,
            name=f"{tube_label}_prism_housing",
        )
        binocular.visual(
            Cylinder(radius=0.018, length=0.06),
            origin=Origin(xyz=(-0.23, tube_y, 0.20)),
            material=anodized_black,
            name=f"{tube_label}_eyepiece",
        )
    binocular.inertial = Inertial.from_geometry(
        Box((1.12, 0.68, 0.34)),
        mass=8.0,
        origin=Origin(xyz=(0.24, 0.30, 0.12)),
    )

    model.articulation(
        "azimuth_bearing",
        ArticulationType.CONTINUOUS,
        parent=tripod,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 1.10)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2),
    )
    model.articulation(
        "altitude_axis",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=binocular,
        origin=Origin(xyz=(0.0, 0.0, 0.52)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=28.0,
            velocity=1.1,
            lower=-0.20,
            upper=1.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    tripod = object_model.get_part("tripod_head")
    yoke = object_model.get_part("yoke_mount")
    binocular = object_model.get_part("binocular_assembly")
    azimuth = object_model.get_articulation("azimuth_bearing")
    altitude = object_model.get_articulation("altitude_axis")

    def aabb_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    with ctx.pose({azimuth: 0.0, altitude: 0.0}):
        ctx.expect_contact(
            yoke,
            tripod,
            elem_a="azimuth_turntable",
            elem_b="bearing_top",
            name="azimuth turntable rests on tripod bearing top",
        )
        ctx.expect_overlap(
            yoke,
            tripod,
            axes="xy",
            elem_a="azimuth_turntable",
            elem_b="bearing_top",
            min_overlap=0.18,
            name="azimuth bearing has overlapping support footprint",
        )
        ctx.expect_contact(
            binocular,
            yoke,
            elem_a="trunnion_plate",
            elem_b="altitude_bearing_plate",
            name="altitude trunnion meets the yoke bearing plate",
        )
        ctx.expect_overlap(
            binocular,
            yoke,
            axes="xz",
            elem_a="trunnion_plate",
            elem_b="altitude_bearing_plate",
            min_overlap=0.12,
            name="altitude joint plates share the same bearing face",
        )

    rest_objective = ctx.part_element_world_aabb(binocular, elem="outer_objective_cell")
    with ctx.pose({altitude: 0.90}):
        raised_objective = ctx.part_element_world_aabb(binocular, elem="outer_objective_cell")
    rest_center = aabb_center(rest_objective)
    raised_center = aabb_center(raised_objective)
    ctx.check(
        "altitude articulation lifts the binocular tubes",
        rest_center is not None
        and raised_center is not None
        and raised_center[2] > rest_center[2] + 0.30
        and raised_center[0] < rest_center[0] - 0.10,
        details=f"rest_center={rest_center}, raised_center={raised_center}",
    )

    with ctx.pose({azimuth: 0.0, altitude: 0.0}):
        rest_objective = ctx.part_element_world_aabb(binocular, elem="outer_objective_cell")
    with ctx.pose({azimuth: math.pi / 2.0, altitude: 0.0}):
        turned_objective = ctx.part_element_world_aabb(binocular, elem="outer_objective_cell")
    rest_center = aabb_center(rest_objective)
    turned_center = aabb_center(turned_objective)
    ctx.check(
        "azimuth articulation swings the binocular assembly around the tripod",
        rest_center is not None
        and turned_center is not None
        and abs(turned_center[0] + rest_center[1]) < 0.08
        and abs(turned_center[1] - rest_center[0]) < 0.08,
        details=f"rest_center={rest_center}, turned_center={turned_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
