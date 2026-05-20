from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lazy_susan_serving_tray", assets=ASSETS)

    tray_wood = model.material("tray_wood", rgba=(0.63, 0.46, 0.28, 1.0))
    base_wood = model.material("base_wood", rgba=(0.37, 0.25, 0.16, 1.0))
    bearing_steel = model.material("bearing_steel", rgba=(0.54, 0.56, 0.58, 1.0))
    felt_black = model.material("felt_black", rgba=(0.10, 0.09, 0.08, 1.0))

    base_shell = _save_mesh(
        "lazy_susan_base_shell.obj",
        LatheGeometry(
            [
                (0.0, 0.0000),
                (0.070, 0.0000),
                (0.125, 0.0015),
                (0.150, 0.0045),
                (0.155, 0.0100),
                (0.155, 0.0175),
                (0.140, 0.0195),
                (0.092, 0.0215),
                (0.050, 0.0220),
                (0.0, 0.0220),
            ],
            segments=72,
        ),
    )
    tray_shell = _save_mesh(
        "lazy_susan_tray_shell.obj",
        LatheGeometry(
            [
                (0.0, 0.0115),
                (0.050, 0.0115),
                (0.110, 0.0100),
                (0.168, 0.0076),
                (0.184, 0.0066),
                (0.190, 0.0098),
                (0.190, 0.0158),
                (0.186, 0.0180),
                (0.175, 0.0190),
                (0.090, 0.0182),
                (0.0, 0.0176),
            ],
            segments=88,
        ),
    )

    base = model.part("base")
    base.visual(base_shell, material=base_wood, name="base_shell")
    base.visual(
        Cylinder(radius=0.058, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=bearing_steel,
        name="lower_bearing_housing",
    )
    base.visual(
        Cylinder(radius=0.046, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=bearing_steel,
        name="lower_bearing_ring",
    )
    base.visual(
        Cylinder(radius=0.105, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=felt_black,
        name="table_pad",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.155, length=0.024),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
    )

    tray = model.part("tray")
    tray.visual(tray_shell, material=tray_wood, name="tray_shell")
    tray.visual(
        Cylinder(radius=0.058, length=0.0095),
        origin=Origin(xyz=(0.0, 0.0, 0.00675)),
        material=bearing_steel,
        name="upper_bearing_housing",
    )
    tray.visual(
        Cylinder(radius=0.046, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=bearing_steel,
        name="upper_bearing_plate",
    )
    tray.inertial = Inertial.from_geometry(
        Cylinder(radius=0.190, length=0.020),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )

    model.articulation(
        "base_to_tray",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=tray,
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    tray = object_model.get_part("tray")
    spin = object_model.get_articulation("base_to_tray")
    base_shell = base.get_visual("base_shell")
    lower_bearing_housing = base.get_visual("lower_bearing_housing")
    lower_bearing_ring = base.get_visual("lower_bearing_ring")
    tray_shell = tray.get_visual("tray_shell")
    upper_bearing_housing = tray.get_visual("upper_bearing_housing")
    upper_bearing_plate = tray.get_visual("upper_bearing_plate")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
    ctx.fail_if_articulation_overlaps(max_pose_samples=8)

    ctx.check(
        "turntable_joint_is_continuous",
        spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"expected continuous joint, got {spin.articulation_type!r}",
    )
    ctx.check(
        "turntable_joint_axis_is_vertical",
        tuple(getattr(spin, "axis", ())) == (0.0, 0.0, 1.0),
        details=f"expected vertical axis, got {getattr(spin, 'axis', None)!r}",
    )

    ctx.expect_origin_distance(tray, base, axes="xy", max_dist=1e-6)
    ctx.expect_overlap(tray, base, axes="xy", min_overlap=0.30)
    ctx.expect_gap(
        tray,
        base,
        axis="z",
        positive_elem=tray_shell,
        negative_elem=base_shell,
        min_gap=0.0075,
        max_gap=0.0105,
        name="shadow_gap_between_discs",
    )
    ctx.expect_contact(
        tray,
        base,
        elem_a=upper_bearing_plate,
        elem_b=lower_bearing_ring,
        contact_tol=1e-6,
        name="bearing_faces_contact",
    )
    ctx.expect_overlap(
        tray,
        base,
        axes="xy",
        elem_a=upper_bearing_plate,
        elem_b=lower_bearing_ring,
        min_overlap=0.08,
        name="bearing_stack_xy_registration",
    )
    ctx.expect_gap(
        tray,
        base,
        axis="z",
        positive_elem=upper_bearing_housing,
        negative_elem=lower_bearing_housing,
        min_gap=0.0035,
        max_gap=0.0045,
        name="bearing_housing_step_gap",
    )

    with ctx.pose({spin: math.pi / 2.0}):
        ctx.expect_origin_distance(tray, base, axes="xy", max_dist=1e-6)
        ctx.expect_contact(
            tray,
            base,
            elem_a=upper_bearing_plate,
            elem_b=lower_bearing_ring,
            contact_tol=1e-6,
            name="bearing_contact_at_quarter_turn",
        )
        ctx.expect_gap(
            tray,
            base,
            axis="z",
            positive_elem=tray_shell,
            negative_elem=base_shell,
            min_gap=0.0075,
            max_gap=0.0105,
            name="shadow_gap_at_quarter_turn",
        )

    with ctx.pose({spin: math.pi}):
        ctx.expect_contact(
            tray,
            base,
            elem_a=upper_bearing_plate,
            elem_b=lower_bearing_ring,
            contact_tol=1e-6,
            name="bearing_contact_at_half_turn",
        )
        ctx.expect_gap(
            tray,
            base,
            axis="z",
            positive_elem=upper_bearing_housing,
            negative_elem=lower_bearing_housing,
            min_gap=0.0035,
            max_gap=0.0045,
            name="bearing_stack_clearance_at_half_turn",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
