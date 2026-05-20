from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _centered_annulus(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .moveTo(inner_radius, -height / 2.0)
        .lineTo(outer_radius, -height / 2.0)
        .lineTo(outer_radius, height / 2.0)
        .lineTo(inner_radius, height / 2.0)
        .close()
        .revolve(360, (0.0, 0.0), (0.0, 1.0))
    )


def _bottom_annulus(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .moveTo(inner_radius, 0.0)
        .lineTo(outer_radius, 0.0)
        .lineTo(outer_radius, height)
        .lineTo(inner_radius, height)
        .close()
        .revolve(360, (0.0, 0.0), (0.0, 1.0))
    )


def _bottom_cylinder(radius: float, height: float, *, bottom: float = 0.0) -> cq.Workplane:
    return cq.Workplane("XY").workplane(offset=bottom).circle(radius).extrude(height)


def _make_base(
    *,
    base_radius: float,
    base_height: float,
    tower_radius: float,
    tower_top: float,
    shaft_radius: float,
    shaft_top: float,
    support_specs: tuple[tuple[float, float, float], ...],
) -> cq.Workplane:
    base = _bottom_cylinder(base_radius, base_height)
    base = base.union(_bottom_cylinder(tower_radius, tower_top))
    base = base.union(_bottom_cylinder(shaft_radius, shaft_top))
    for bottom, height, radius in support_specs:
        base = base.union(_bottom_cylinder(radius, height, bottom=bottom))
    return base


def _make_rotary_stage(
    *,
    outer_radius: float,
    bore_radius: float,
    hub_radius: float,
    plate_thickness: float,
    hub_height: float,
    tab_length: float,
    tab_width: float,
) -> cq.Workplane:
    plate = _bottom_annulus(outer_radius, bore_radius, plate_thickness)
    hub = _bottom_annulus(hub_radius, bore_radius, hub_height)
    rim_band = (
        _centered_annulus(outer_radius * 0.92, hub_radius + 0.010, 0.003)
        .translate((0.0, 0.0, plate_thickness - 0.0015))
    )
    tab = (
        cq.Workplane("XY")
        .box(tab_length, tab_width, plate_thickness)
        .translate((outer_radius + tab_length / 2.0 - 0.004, 0.0, plate_thickness / 2.0))
    )
    stage = plate.union(hub).union(rim_band).union(tab)
    return stage


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stepped_coaxial_rotary_stack")

    model.material("base_finish", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("spindle_finish", rgba=(0.30, 0.32, 0.35, 1.0))
    model.material("stage_large_finish", rgba=(0.70, 0.72, 0.74, 1.0))
    model.material("stage_mid_finish", rgba=(0.58, 0.61, 0.65, 1.0))
    model.material("stage_small_finish", rgba=(0.47, 0.52, 0.58, 1.0))
    model.material("stage_top_finish", rgba=(0.76, 0.77, 0.79, 1.0))

    base_radius = 0.180
    base_height = 0.036
    tower_radius = 0.068
    tower_top = 0.056
    shaft_radius = 0.022
    shaft_top = 0.238
    bore_radius = 0.030
    spindle_origin_z = tower_top

    stage_specs = (
        {
            "part_name": "stage_large",
            "joint_name": "spindle_to_stage_large",
            "mesh_name": "stage_large",
            "material": "stage_large_finish",
            "bottom_z": 0.066,
            "outer_radius": 0.140,
            "hub_radius": 0.064,
            "plate_thickness": 0.012,
            "hub_height": 0.018,
            "tab_length": 0.030,
            "tab_width": 0.026,
            "support_radius": 0.056,
            "support_height": 0.010,
        },
        {
            "part_name": "stage_mid",
            "joint_name": "spindle_to_stage_mid",
            "mesh_name": "stage_mid",
            "material": "stage_mid_finish",
            "bottom_z": 0.112,
            "outer_radius": 0.112,
            "hub_radius": 0.056,
            "plate_thickness": 0.011,
            "hub_height": 0.016,
            "tab_length": 0.028,
            "tab_width": 0.022,
            "support_radius": 0.048,
            "support_height": 0.010,
        },
        {
            "part_name": "stage_small",
            "joint_name": "spindle_to_stage_small",
            "mesh_name": "stage_small",
            "material": "stage_small_finish",
            "bottom_z": 0.155,
            "outer_radius": 0.086,
            "hub_radius": 0.047,
            "plate_thickness": 0.009,
            "hub_height": 0.014,
            "tab_length": 0.022,
            "tab_width": 0.018,
            "support_radius": 0.041,
            "support_height": 0.010,
        },
        {
            "part_name": "stage_top",
            "joint_name": "spindle_to_stage_top",
            "mesh_name": "stage_top",
            "material": "stage_top_finish",
            "bottom_z": 0.196,
            "outer_radius": 0.062,
            "hub_radius": 0.039,
            "plate_thickness": 0.008,
            "hub_height": 0.012,
            "tab_length": 0.018,
            "tab_width": 0.015,
            "support_radius": 0.034,
            "support_height": 0.010,
        },
    )

    base = model.part("base")
    base.visual(
        Cylinder(radius=base_radius, length=base_height),
        origin=Origin(xyz=(0.0, 0.0, base_height / 2.0)),
        material="base_finish",
        name="base_foot",
    )
    base.visual(
        Cylinder(radius=tower_radius, length=tower_top - base_height),
        origin=Origin(xyz=(0.0, 0.0, (tower_top + base_height) / 2.0)),
        material="base_finish",
        name="base_tower",
    )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=shaft_radius, length=shaft_top - spindle_origin_z),
        origin=Origin(xyz=(0.0, 0.0, (shaft_top - spindle_origin_z) / 2.0)),
        material="spindle_finish",
        name="shaft_body",
    )
    for spec in stage_specs:
        collar_bottom_local = spec["bottom_z"] - spindle_origin_z - spec["support_height"]
        spindle.visual(
            Cylinder(radius=spec["support_radius"], length=spec["support_height"]),
            origin=Origin(
                xyz=(
                    0.0,
                    0.0,
                    collar_bottom_local + spec["support_height"] / 2.0,
                )
            ),
            material="spindle_finish",
            name=f"{spec['part_name']}_collar",
        )

    model.articulation(
        "base_to_spindle",
        ArticulationType.FIXED,
        parent=base,
        child=spindle,
        origin=Origin(xyz=(0.0, 0.0, spindle_origin_z)),
    )

    for spec in stage_specs:
        stage_shape = _make_rotary_stage(
            outer_radius=spec["outer_radius"],
            bore_radius=bore_radius,
            hub_radius=spec["hub_radius"],
            plate_thickness=spec["plate_thickness"],
            hub_height=spec["hub_height"],
            tab_length=spec["tab_length"],
            tab_width=spec["tab_width"],
        )
        stage = model.part(spec["part_name"])
        stage.visual(
            mesh_from_cadquery(stage_shape, spec["mesh_name"]),
            origin=Origin(),
            material=spec["material"],
            name=f"{spec['part_name']}_body",
        )
        model.articulation(
            spec["joint_name"],
            ArticulationType.REVOLUTE,
            parent=spindle,
            child=stage,
            origin=Origin(xyz=(0.0, 0.0, spec["bottom_z"] - spindle_origin_z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=2.5,
                lower=-pi,
                upper=pi,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    spindle = object_model.get_part("spindle")
    stage_large = object_model.get_part("stage_large")
    stage_mid = object_model.get_part("stage_mid")
    stage_small = object_model.get_part("stage_small")
    stage_top = object_model.get_part("stage_top")
    joints = [
        object_model.get_articulation("spindle_to_stage_large"),
        object_model.get_articulation("spindle_to_stage_mid"),
        object_model.get_articulation("spindle_to_stage_small"),
        object_model.get_articulation("spindle_to_stage_top"),
    ]

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    for stage in (stage_large, stage_mid, stage_small, stage_top):
        ctx.allow_overlap(
            spindle,
            stage,
            reason="Rotary stages are intentionally sleeved onto the shared vertical spindle with negligible modeled bearing clearance.",
        )

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

    for part_name in ("base", "spindle", "stage_large", "stage_mid", "stage_small", "stage_top"):
        ctx.check(
            f"{part_name}_present",
            object_model.get_part(part_name) is not None,
            details=f"Missing required part {part_name}",
        )

    ctx.expect_contact(
        spindle,
        base,
        contact_tol=0.001,
        name="spindle_mounted_to_base",
    )

    for stage in (stage_large, stage_mid, stage_small, stage_top):
        ctx.expect_contact(
            stage,
            spindle,
            contact_tol=0.001,
            name=f"{stage.name}_supported_by_spindle",
        )
        ctx.expect_overlap(
            stage,
            spindle,
            axes="xy",
            min_overlap=0.04,
            name=f"{stage.name}_coaxial_over_spindle",
        )
        ctx.expect_gap(
            stage,
            base,
            axis="z",
            min_gap=0.008,
            name=f"{stage.name}_clear_of_base_plinth",
        )

    ctx.expect_gap(
        stage_mid,
        stage_large,
        axis="z",
        min_gap=0.020,
        name="large_to_mid_vertical_separation",
    )
    ctx.expect_gap(
        stage_small,
        stage_mid,
        axis="z",
        min_gap=0.020,
        name="mid_to_small_vertical_separation",
    )
    ctx.expect_gap(
        stage_top,
        stage_small,
        axis="z",
        min_gap=0.020,
        name="small_to_top_vertical_separation",
    )

    for lower, upper, name_prefix in (
        (stage_large, stage_mid, "large_mid"),
        (stage_mid, stage_small, "mid_small"),
        (stage_small, stage_top, "small_top"),
    ):
        ctx.expect_overlap(
            lower,
            upper,
            axes="xy",
            min_overlap=0.08,
            name=f"{name_prefix}_shared_stack_axis",
        )

    base_to_spindle = object_model.get_articulation("base_to_spindle")
    ctx.check(
        "base_to_spindle_centered",
        abs(base_to_spindle.origin.xyz[0]) < 1e-9
        and abs(base_to_spindle.origin.xyz[1]) < 1e-9
        and abs(base_to_spindle.origin.xyz[2] - 0.056) < 1e-9,
        details=f"base_to_spindle origin was {base_to_spindle.origin.xyz}",
    )

    expected_z = [0.010, 0.056, 0.099, 0.140]
    for joint, z_value in zip(joints, expected_z):
        ctx.check(
            f"{joint.name}_vertical_axis",
            tuple(round(v, 6) for v in joint.axis) == (0.0, 0.0, 1.0),
            details=f"{joint.name} axis was {joint.axis}",
        )
        ctx.check(
            f"{joint.name}_shared_centerline",
            abs(joint.origin.xyz[0]) < 1e-9 and abs(joint.origin.xyz[1]) < 1e-9,
            details=f"{joint.name} origin xy was {joint.origin.xyz[:2]}",
        )
        ctx.check(
            f"{joint.name}_stacked_height",
            abs(joint.origin.xyz[2] - z_value) < 1e-9,
            details=f"{joint.name} origin z was {joint.origin.xyz[2]} expected {z_value}",
        )

    with ctx.pose(
        spindle_to_stage_large=pi / 2.0,
        spindle_to_stage_mid=-pi / 3.0,
        spindle_to_stage_small=pi / 4.0,
        spindle_to_stage_top=-pi / 2.0,
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="rotated_pose_clearance")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
