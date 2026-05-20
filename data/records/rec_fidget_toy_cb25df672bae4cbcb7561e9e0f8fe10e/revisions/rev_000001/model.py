from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import isclose, pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


BODY_SIZE = (0.060, 0.055, 0.055)
BODY_CORNER_RADIUS = 0.0075
TOP_PANEL_SIZE = (0.044, 0.044, 0.002)
DIAL_PANEL_SIZE = (0.002, 0.034, 0.034)
DIAL_PANEL_EMBED = 0.0006
SLIDER_PANEL_SIZE = (0.038, 0.003, 0.022)
SLIDER_PANEL_Z = -0.008
SLIDER_PANEL_EMBED = 0.0006

JOYSTICK_PIVOT_RADIUS = 0.0045
JOYSTICK_STEM_RADIUS = 0.0032
JOYSTICK_STEM_LENGTH = 0.016
JOYSTICK_NUB_RADIUS = 0.0095
JOYSTICK_STEM_CENTER_Z = 0.011
JOYSTICK_NUB_CENTER_Z = 0.0235

DIAL_HUB_RADIUS = 0.0045
DIAL_HUB_LENGTH = 0.004
DIAL_RADIUS = 0.014
DIAL_LENGTH = 0.008
DIAL_CENTER_X = 0.006
DIAL_MARKER_SIZE = (0.0025, 0.004, 0.0025)
DIAL_MARKER_X = 0.0105
DIAL_MARKER_Z = 0.0105

SLIDER_BASE_SIZE = (0.011, 0.006, 0.010)
SLIDER_BASE_RADIUS = 0.0022
SLIDER_TAB_SIZE = (0.016, 0.004, 0.006)
SLIDER_TAB_RADIUS = 0.0018
SLIDER_TAB_CENTER_Y = 0.0045


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fidget_cube")

    body_shell = model.material("body_shell", rgba=(0.94, 0.94, 0.92, 1.0))
    face_panel = model.material("face_panel", rgba=(0.22, 0.24, 0.27, 1.0))
    joystick_color = model.material("joystick_color", rgba=(0.83, 0.36, 0.18, 1.0))
    dial_color = model.material("dial_color", rgba=(0.13, 0.14, 0.16, 1.0))
    slider_color = model.material("slider_color", rgba=(0.20, 0.56, 0.84, 1.0))
    highlight = model.material("highlight", rgba=(0.95, 0.77, 0.20, 1.0))

    body = model.part("body")
    body_shell_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(
                BODY_SIZE[0], BODY_SIZE[1], BODY_CORNER_RADIUS, corner_segments=10
            ),
            BODY_SIZE[2],
            cap=True,
            center=True,
        ),
        "fidget_cube_body_shell",
    )
    body.visual(body_shell_mesh, material=body_shell, name="main_shell")
    body.visual(
        Box(TOP_PANEL_SIZE),
        origin=Origin(xyz=(0.0, 0.0, BODY_SIZE[2] / 2 + TOP_PANEL_SIZE[2] / 2)),
        material=face_panel,
        name="joystick_panel",
    )
    body.visual(
        Box(DIAL_PANEL_SIZE),
        origin=Origin(
            xyz=(BODY_SIZE[0] / 2 + DIAL_PANEL_SIZE[0] / 2 - DIAL_PANEL_EMBED, 0.0, 0.0)
        ),
        material=face_panel,
        name="dial_panel",
    )
    body.visual(
        Box(SLIDER_PANEL_SIZE),
        origin=Origin(
            xyz=(
                0.0,
                BODY_SIZE[1] / 2 + SLIDER_PANEL_SIZE[1] / 2 - SLIDER_PANEL_EMBED,
                SLIDER_PANEL_Z,
            )
        ),
        material=face_panel,
        name="slider_panel",
    )
    body.inertial = Inertial.from_geometry(Box(BODY_SIZE), mass=0.18, origin=Origin())

    joystick = model.part("joystick")
    joystick.visual(
        Sphere(JOYSTICK_PIVOT_RADIUS),
        material=face_panel,
        name="pivot_ball",
    )
    joystick.visual(
        Cylinder(radius=JOYSTICK_STEM_RADIUS, length=JOYSTICK_STEM_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, JOYSTICK_STEM_CENTER_Z)),
        material=joystick_color,
        name="stem",
    )
    joystick.visual(
        Sphere(JOYSTICK_NUB_RADIUS),
        origin=Origin(xyz=(0.0, 0.0, JOYSTICK_NUB_CENTER_Z)),
        material=joystick_color,
        name="thumb_nub",
    )
    joystick.inertial = Inertial.from_geometry(
        Sphere(JOYSTICK_NUB_RADIUS),
        mass=0.02,
        origin=Origin(xyz=(0.0, 0.0, JOYSTICK_NUB_CENTER_Z)),
    )

    dial = model.part("dial_wheel")
    dial.visual(
        Cylinder(radius=DIAL_HUB_RADIUS, length=DIAL_HUB_LENGTH),
        origin=Origin(rpy=(0.0, pi / 2, 0.0)),
        material=face_panel,
        name="hub",
    )
    dial.visual(
        Cylinder(radius=DIAL_RADIUS, length=DIAL_LENGTH),
        origin=Origin(xyz=(DIAL_CENTER_X, 0.0, 0.0), rpy=(0.0, pi / 2, 0.0)),
        material=dial_color,
        name="wheel_disc",
    )
    dial.visual(
        Box(DIAL_MARKER_SIZE),
        origin=Origin(xyz=(DIAL_MARKER_X, 0.0, DIAL_MARKER_Z)),
        material=highlight,
        name="wheel_marker",
    )
    dial.inertial = Inertial.from_geometry(
        Cylinder(radius=DIAL_RADIUS, length=DIAL_LENGTH),
        mass=0.015,
        origin=Origin(xyz=(DIAL_CENTER_X, 0.0, 0.0), rpy=(0.0, pi / 2, 0.0)),
    )

    slider = model.part("slider_switch")
    slider_base_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(
                SLIDER_BASE_SIZE[0],
                SLIDER_BASE_SIZE[1],
                SLIDER_BASE_RADIUS,
                corner_segments=8,
            ),
            SLIDER_BASE_SIZE[2],
            cap=True,
            center=True,
        ),
        "slider_switch_base",
    )
    slider_tab_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(
                SLIDER_TAB_SIZE[0],
                SLIDER_TAB_SIZE[1],
                SLIDER_TAB_RADIUS,
                corner_segments=8,
            ),
            SLIDER_TAB_SIZE[2],
            cap=True,
            center=True,
        ),
        "slider_switch_tab",
    )
    slider.visual(slider_base_mesh, material=slider_color, name="slider_base")
    slider.visual(
        slider_tab_mesh,
        origin=Origin(xyz=(0.0, SLIDER_TAB_CENTER_Y, 0.0)),
        material=highlight,
        name="slider_tab",
    )
    slider.inertial = Inertial.from_geometry(
        Box(SLIDER_BASE_SIZE),
        mass=0.01,
        origin=Origin(),
    )

    model.articulation(
        "body_to_joystick",
        ArticulationType.REVOLUTE,
        parent=body,
        child=joystick,
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                BODY_SIZE[2] / 2 + TOP_PANEL_SIZE[2] + JOYSTICK_PIVOT_RADIUS,
            )
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=2.5,
            lower=-0.45,
            upper=0.45,
        ),
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.REVOLUTE,
        parent=body,
        child=dial,
        origin=Origin(
            xyz=(
                BODY_SIZE[0] / 2
                + DIAL_PANEL_SIZE[0]
                + DIAL_HUB_LENGTH / 2
                - DIAL_PANEL_EMBED,
                0.0,
                0.0,
            )
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=8.0,
            lower=-pi,
            upper=pi,
        ),
    )
    model.articulation(
        "body_to_slider",
        ArticulationType.PRISMATIC,
        parent=body,
        child=slider,
        origin=Origin(
            xyz=(
                0.0,
                BODY_SIZE[1] / 2
                + SLIDER_PANEL_SIZE[1]
                + SLIDER_BASE_SIZE[1] / 2
                - SLIDER_PANEL_EMBED,
                SLIDER_PANEL_Z,
            )
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.15,
            lower=-0.006,
            upper=0.006,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, seed=0)
    body = object_model.get_part("body")
    joystick = object_model.get_part("joystick")
    dial = object_model.get_part("dial_wheel")
    slider = object_model.get_part("slider_switch")

    joystick_joint = object_model.get_articulation("body_to_joystick")
    dial_joint = object_model.get_articulation("body_to_dial")
    slider_joint = object_model.get_articulation("body_to_slider")

    main_shell = body.get_visual("main_shell")
    joystick_panel = body.get_visual("joystick_panel")
    dial_panel = body.get_visual("dial_panel")
    slider_panel = body.get_visual("slider_panel")

    def shell_size_ok() -> bool:
        shell_aabb = ctx.part_element_world_aabb(body, elem=main_shell)
        if shell_aabb is None:
            return False
        mins, maxs = shell_aabb
        dims = tuple(maxs[i] - mins[i] for i in range(3))
        return all(isclose(dims[i], BODY_SIZE[i], abs_tol=1e-6) for i in range(3))

    def check_control_pose(
        joint,
        position: float,
        child,
        *,
        axis: str,
        overlap_axes: str,
        min_overlap: float,
        outer_elem,
        pose_label: str,
    ) -> None:
        with ctx.pose({joint: position}):
            ctx.fail_if_parts_overlap_in_current_pose(
                name=f"{joint.name}_{pose_label}_no_overlap"
            )
            ctx.fail_if_isolated_parts(name=f"{joint.name}_{pose_label}_no_floating")
            ctx.expect_contact(child, body, name=f"{joint.name}_{pose_label}_contact")
            ctx.expect_gap(
                child,
                body,
                axis=axis,
                max_gap=1e-6,
                max_penetration=1e-6,
                name=f"{joint.name}_{pose_label}_mounted_gap",
            )
            ctx.expect_overlap(
                child,
                body,
                axes=overlap_axes,
                min_overlap=min_overlap,
                name=f"{joint.name}_{pose_label}_face_overlap",
            )
            ctx.expect_within(
                child,
                body,
                axes=overlap_axes,
                outer_elem=outer_elem,
                margin=0.0,
                name=f"{joint.name}_{pose_label}_within_panel",
            )

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=36)

    ctx.check(
        "expected_parts_present",
        len(object_model.parts) == 4,
        f"expected 4 parts, found {len(object_model.parts)}",
    )
    ctx.check(
        "body_shell_dimensions",
        shell_size_ok(),
        f"body shell should be exactly {BODY_SIZE}",
    )
    ctx.check(
        "joystick_joint_type_and_axis",
        joystick_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 6) for v in joystick_joint.axis) == (1.0, 0.0, 0.0),
        f"expected revolute joystick about +x, got {joystick_joint.articulation_type} {joystick_joint.axis}",
    )
    ctx.check(
        "dial_joint_type_and_axis",
        dial_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 6) for v in dial_joint.axis) == (1.0, 0.0, 0.0),
        f"expected revolute dial about +x, got {dial_joint.articulation_type} {dial_joint.axis}",
    )
    ctx.check(
        "slider_joint_type_and_axis",
        slider_joint.articulation_type == ArticulationType.PRISMATIC
        and tuple(round(v, 6) for v in slider_joint.axis) == (1.0, 0.0, 0.0),
        f"expected prismatic slider along +x, got {slider_joint.articulation_type} {slider_joint.axis}",
    )

    check_control_pose(
        joystick_joint,
        0.0,
        joystick,
        axis="z",
        overlap_axes="xy",
        min_overlap=0.009,
        outer_elem=joystick_panel,
        pose_label="rest",
    )
    check_control_pose(
        dial_joint,
        0.0,
        dial,
        axis="x",
        overlap_axes="yz",
        min_overlap=0.010,
        outer_elem=dial_panel,
        pose_label="rest",
    )
    check_control_pose(
        slider_joint,
        0.0,
        slider,
        axis="y",
        overlap_axes="xz",
        min_overlap=0.008,
        outer_elem=slider_panel,
        pose_label="rest",
    )

    for joint, child, axis, overlap_axes, min_overlap, outer_elem in (
        (joystick_joint, joystick, "z", "xy", 0.009, joystick_panel),
        (dial_joint, dial, "x", "yz", 0.010, dial_panel),
        (slider_joint, slider, "y", "xz", 0.008, slider_panel),
    ):
        limits = joint.motion_limits
        if limits is not None and limits.lower is not None and limits.upper is not None:
            check_control_pose(
                joint,
                limits.lower,
                child,
                axis=axis,
                overlap_axes=overlap_axes,
                min_overlap=min_overlap,
                outer_elem=outer_elem,
                pose_label="lower",
            )
            check_control_pose(
                joint,
                limits.upper,
                child,
                axis=axis,
                overlap_axes=overlap_axes,
                min_overlap=min_overlap,
                outer_elem=outer_elem,
                pose_label="upper",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
