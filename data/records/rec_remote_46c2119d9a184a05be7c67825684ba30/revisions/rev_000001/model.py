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
    CylinderGeometry,
    ExtrudeGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def _interp_dims(
    x: float,
    control_sections: list[tuple[float, float, float, float]],
) -> tuple[float, float, float]:
    if x <= control_sections[0][0]:
        _, width, top, bottom = control_sections[0]
        return width, top, bottom
    if x >= control_sections[-1][0]:
        _, width, top, bottom = control_sections[-1]
        return width, top, bottom

    for index in range(len(control_sections) - 1):
        x0, w0, top0, bottom0 = control_sections[index]
        x1, w1, top1, bottom1 = control_sections[index + 1]
        if x0 <= x <= x1:
            t = (x - x0) / (x1 - x0)
            return (
                _lerp(w0, w1, t),
                _lerp(top0, top1, t),
                _lerp(bottom0, bottom1, t),
            )

    _, width, top, bottom = control_sections[-1]
    return width, top, bottom


def _section_loop(
    x: float,
    width: float,
    top_height: float,
    bottom_height: float,
    samples: int,
) -> list[tuple[float, float, float]]:
    loop: list[tuple[float, float, float]] = []
    for idx in range(samples):
        theta = math.tau * idx / samples
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        y = 0.5 * width * math.copysign(abs(cos_t) ** 0.72, cos_t)
        if sin_t >= 0.0:
            z = top_height * (sin_t ** 0.82)
        else:
            z = -bottom_height * ((-sin_t) ** 1.28)
        loop.append((x, y, z))
    return loop


def _add_quad(mesh: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    mesh.add_face(a, b, c)
    mesh.add_face(a, c, d)


def _build_body_shell() -> MeshGeometry:
    control_sections = [
        (-0.074, 0.020, 0.0058, 0.0032),
        (-0.056, 0.034, 0.0089, 0.0042),
        (-0.020, 0.045, 0.0115, 0.0052),
        (0.020, 0.050, 0.0126, 0.0060),
        (0.052, 0.046, 0.0111, 0.0056),
        (0.074, 0.026, 0.0067, 0.0037),
    ]
    wall = 0.0017
    section_count = 29
    loop_samples = 44
    x_min = control_sections[0][0]
    x_max = control_sections[-1][0]

    x_positions = [
        _lerp(x_min, x_max, idx / (section_count - 1)) for idx in range(section_count)
    ]
    dims_along_x = [_interp_dims(x, control_sections) for x in x_positions]
    outer_loops = [
        _section_loop(x, width, top, bottom, loop_samples)
        for x, (width, top, bottom) in zip(x_positions, dims_along_x)
    ]
    inner_loops = [
        _section_loop(
            x,
            max(width - 2.0 * wall, 0.008),
            max(top - wall, 0.0015),
            max(bottom - 0.9 * wall, 0.0012),
            loop_samples,
        )
        for x, (width, top, bottom) in zip(x_positions, dims_along_x)
    ]

    opening_x0 = -0.020
    opening_x1 = 0.044
    opening_width = 0.035

    mesh = MeshGeometry()
    outer_idx: list[list[int]] = []
    inner_idx: list[list[int]] = []

    for loop in outer_loops:
        outer_idx.append([mesh.add_vertex(*point) for point in loop])
    for loop in inner_loops:
        inner_idx.append([mesh.add_vertex(*point) for point in loop])

    omitted: list[list[bool]] = []
    for ix in range(section_count - 1):
        cell_row: list[bool] = []
        width0, _, bottom0 = dims_along_x[ix]
        width1, _, bottom1 = dims_along_x[ix + 1]
        avg_bottom = 0.5 * (bottom0 + bottom1)
        avg_width = 0.5 * (width0 + width1)
        for it in range(loop_samples):
            nxt = (it + 1) % loop_samples
            pts = (
                outer_loops[ix][it],
                outer_loops[ix][nxt],
                outer_loops[ix + 1][nxt],
                outer_loops[ix + 1][it],
            )
            center_x = sum(pt[0] for pt in pts) / 4.0
            center_y = sum(pt[1] for pt in pts) / 4.0
            center_z = sum(pt[2] for pt in pts) / 4.0
            cell_row.append(
                opening_x0 <= center_x <= opening_x1
                and abs(center_y) <= min(opening_width * 0.5, avg_width * 0.42)
                and center_z <= -0.58 * avg_bottom
            )
        omitted.append(cell_row)

    for ix in range(section_count - 1):
        for it in range(loop_samples):
            nxt = (it + 1) % loop_samples
            if not omitted[ix][it]:
                _add_quad(
                    mesh,
                    outer_idx[ix][it],
                    outer_idx[ix][nxt],
                    outer_idx[ix + 1][nxt],
                    outer_idx[ix + 1][it],
                )
                _add_quad(
                    mesh,
                    inner_idx[ix][it],
                    inner_idx[ix + 1][it],
                    inner_idx[ix + 1][nxt],
                    inner_idx[ix][nxt],
                )

    for it in range(loop_samples):
        nxt = (it + 1) % loop_samples
        _add_quad(
            mesh,
            outer_idx[0][it],
            inner_idx[0][it],
            inner_idx[0][nxt],
            outer_idx[0][nxt],
        )
        _add_quad(
            mesh,
            outer_idx[-1][it],
            outer_idx[-1][nxt],
            inner_idx[-1][nxt],
            inner_idx[-1][it],
        )

    for ix in range(section_count - 1):
        for it in range(loop_samples):
            if not omitted[ix][it]:
                continue
            nxt = (it + 1) % loop_samples

            if ix == 0 or not omitted[ix - 1][it]:
                _add_quad(
                    mesh,
                    outer_idx[ix][it],
                    inner_idx[ix][it],
                    inner_idx[ix][nxt],
                    outer_idx[ix][nxt],
                )
            if ix == section_count - 2 or not omitted[ix + 1][it]:
                _add_quad(
                    mesh,
                    outer_idx[ix + 1][it],
                    outer_idx[ix + 1][nxt],
                    inner_idx[ix + 1][nxt],
                    inner_idx[ix + 1][it],
                )
            prev_t = (it - 1) % loop_samples
            if not omitted[ix][prev_t]:
                _add_quad(
                    mesh,
                    outer_idx[ix][it],
                    outer_idx[ix + 1][it],
                    inner_idx[ix + 1][it],
                    inner_idx[ix][it],
                )
            if not omitted[ix][nxt]:
                _add_quad(
                    mesh,
                    outer_idx[ix][nxt],
                    inner_idx[ix][nxt],
                    inner_idx[ix + 1][nxt],
                    outer_idx[ix + 1][nxt],
                )

    return mesh


def _build_battery_cover_mesh(
    cover_length: float,
    cover_width: float,
    cover_thickness: float,
) -> MeshGeometry:
    cover = ExtrudeGeometry(
        rounded_rect_profile(cover_length, cover_width, radius=0.0055, corner_segments=8),
        cover_thickness,
        center=True,
    )
    cover.translate(cover_length * 0.5, 0.0, -cover_thickness * 0.5)
    return cover


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="air_mouse_remote")

    shell_charcoal = model.material("shell_charcoal", rgba=(0.12, 0.13, 0.15, 1.0))
    soft_black = model.material("soft_black", rgba=(0.06, 0.06, 0.07, 1.0))
    satin_graphite = model.material("satin_graphite", rgba=(0.22, 0.23, 0.25, 1.0))
    warm_gray = model.material("warm_gray", rgba=(0.26, 0.27, 0.29, 1.0))
    indicator_gray = model.material("indicator_gray", rgba=(0.72, 0.73, 0.75, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_geometry(_build_body_shell(), "remote_body_shell"),
        material=shell_charcoal,
        name="body_shell",
    )
    body.visual(
        Box((0.022, 0.030, 0.0032)),
        origin=Origin(xyz=(0.014, 0.0, 0.0104)),
        material=shell_charcoal,
        name="dial_bridge",
    )
    body.visual(
        Box((0.008, 0.0048, 0.010)),
        origin=Origin(xyz=(0.014, 0.0142, 0.0132)),
        material=shell_charcoal,
        name="dial_cheek_left",
    )
    body.visual(
        Box((0.008, 0.0048, 0.010)),
        origin=Origin(xyz=(0.014, -0.0142, 0.0132)),
        material=shell_charcoal,
        name="dial_cheek_right",
    )
    body.visual(
        Box((0.018, 0.021, 0.0026)),
        origin=Origin(xyz=(0.014, 0.0, 0.0089)),
        material=warm_gray,
        name="dial_saddle",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.148, 0.050, 0.024)),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
    )

    jog_dial = model.part("jog_dial")
    jog_dial.visual(
        Cylinder(radius=0.0072, length=0.024),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=soft_black,
        name="dial_tire",
    )
    jog_dial.visual(
        Cylinder(radius=0.0078, length=0.004),
        origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=satin_graphite,
        name="dial_flange_left",
    )
    jog_dial.visual(
        Cylinder(radius=0.0078, length=0.004),
        origin=Origin(xyz=(0.0, -0.010, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=satin_graphite,
        name="dial_flange_right",
    )
    jog_dial.visual(
        Box((0.0055, 0.006, 0.0018)),
        origin=Origin(xyz=(0.0, 0.0, 0.0071)),
        material=indicator_gray,
        name="dial_marker",
    )
    jog_dial.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0075, length=0.024),
        mass=0.01,
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
    )

    battery_cover = model.part("battery_cover")
    cover_length = 0.064
    cover_width = 0.034
    cover_thickness = 0.0022
    hinge_barrel_radius = 0.0025
    hinge_center_span = 0.011
    hinge_outer_span = 0.010
    hinge_side_offset = 0.0105
    battery_cover.visual(
        mesh_from_geometry(
            _build_battery_cover_mesh(cover_length, cover_width, cover_thickness),
            "battery_cover_panel",
        ),
        material=satin_graphite,
        name="cover_panel",
    )
    battery_cover.visual(
        Cylinder(radius=hinge_barrel_radius, length=hinge_center_span),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=soft_black,
        name="cover_hinge_barrel",
    )
    battery_cover.visual(
        Box((0.0045, 0.014, 0.0036)),
        origin=Origin(xyz=(0.00225, 0.0, -0.0018)),
        material=satin_graphite,
        name="cover_hinge_strap",
    )
    battery_cover.visual(
        Box((0.010, 0.012, 0.0013)),
        origin=Origin(xyz=(cover_length - 0.004, 0.0, -0.0018)),
        material=warm_gray,
        name="cover_lip",
    )
    battery_cover.inertial = Inertial.from_geometry(
        Box((cover_length, cover_width, cover_thickness)),
        mass=0.018,
        origin=Origin(xyz=(cover_length * 0.5, 0.0, -cover_thickness * 0.5)),
    )

    model.articulation(
        "body_to_jog_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=jog_dial,
        origin=Origin(xyz=(0.014, 0.0, 0.0157)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.35, velocity=20.0),
    )
    body.visual(
        Box((0.0065, 0.008, 0.0052)),
        origin=Origin(xyz=(-0.0183, hinge_side_offset, -0.0062)),
        material=shell_charcoal,
        name="hinge_mount_left",
    )
    body.visual(
        Box((0.0065, 0.008, 0.0052)),
        origin=Origin(xyz=(-0.0183, -hinge_side_offset, -0.0062)),
        material=shell_charcoal,
        name="hinge_mount_right",
    )
    body.visual(
        Cylinder(radius=hinge_barrel_radius, length=hinge_outer_span),
        origin=Origin(
            xyz=(-0.020, hinge_side_offset, -0.0060),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=soft_black,
        name="hinge_barrel_left",
    )
    body.visual(
        Cylinder(radius=hinge_barrel_radius, length=hinge_outer_span),
        origin=Origin(
            xyz=(-0.020, -hinge_side_offset, -0.0060),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=soft_black,
        name="hinge_barrel_right",
    )
    model.articulation(
        "body_to_battery_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=battery_cover,
        origin=Origin(xyz=(-0.020, 0.0, -0.0060)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.25,
            velocity=2.0,
            lower=0.0,
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
    body = object_model.get_part("body")
    jog_dial = object_model.get_part("jog_dial")
    battery_cover = object_model.get_part("battery_cover")
    dial_joint = object_model.get_articulation("body_to_jog_dial")
    cover_joint = object_model.get_articulation("body_to_battery_cover")

    ctx.check(
        "jog dial uses continuous horizontal-axis rotation",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS
        and dial_joint.motion_limits is not None
        and dial_joint.motion_limits.lower is None
        and dial_joint.motion_limits.upper is None
        and abs(dial_joint.axis[0]) < 1e-6
        and abs(dial_joint.axis[1] - 1.0) < 1e-6
        and abs(dial_joint.axis[2]) < 1e-6,
        details=f"type={dial_joint.articulation_type}, axis={dial_joint.axis}, limits={dial_joint.motion_limits}",
    )
    ctx.check(
        "battery cover hinges from a short edge",
        cover_joint.articulation_type == ArticulationType.REVOLUTE
        and cover_joint.motion_limits is not None
        and cover_joint.motion_limits.lower is not None
        and cover_joint.motion_limits.upper is not None
        and cover_joint.motion_limits.upper > 1.2
        and abs(cover_joint.axis[0]) < 1e-6
        and abs(cover_joint.axis[1] - 1.0) < 1e-6
        and abs(cover_joint.axis[2]) < 1e-6,
        details=f"type={cover_joint.articulation_type}, axis={cover_joint.axis}, limits={cover_joint.motion_limits}",
    )

    with ctx.pose({cover_joint: 0.0}):
        ctx.expect_contact(
            body,
            battery_cover,
            elem_a="hinge_barrel_left",
            elem_b="cover_hinge_barrel",
            name="left hinge knuckle stays connected",
        )
        ctx.expect_contact(
            body,
            battery_cover,
            elem_a="hinge_barrel_right",
            elem_b="cover_hinge_barrel",
            name="right hinge knuckle stays connected",
        )
        ctx.expect_gap(
            body,
            battery_cover,
            axis="z",
            positive_elem="body_shell",
            negative_elem="cover_panel",
            min_gap=0.0,
            max_gap=0.0012,
            name="battery cover sits flush with the underside",
        )
        ctx.expect_within(
            battery_cover,
            body,
            axes="xy",
            margin=0.012,
            name="battery cover stays within the remote footprint",
        )

    with ctx.pose({cover_joint: 1.25}):
        ctx.expect_gap(
            body,
            battery_cover,
            axis="z",
            negative_elem="cover_lip",
            min_gap=0.020,
            name="battery cover swings down away from the body",
        )

    rest_marker = ctx.part_element_world_aabb(jog_dial, elem="dial_marker")
    with ctx.pose({dial_joint: math.pi * 0.5}):
        turned_marker = ctx.part_element_world_aabb(jog_dial, elem="dial_marker")

    marker_rotates_forward = False
    if rest_marker is not None and turned_marker is not None:
        rest_center = tuple(
            0.5 * (rest_marker[0][axis] + rest_marker[1][axis]) for axis in range(3)
        )
        turned_center = tuple(
            0.5 * (turned_marker[0][axis] + turned_marker[1][axis]) for axis in range(3)
        )
        marker_rotates_forward = (
            turned_center[0] > rest_center[0] + 0.004
            and turned_center[2] < rest_center[2] - 0.004
        )
        detail = f"rest_center={rest_center}, turned_center={turned_center}"
    else:
        detail = f"rest_marker={rest_marker}, turned_marker={turned_marker}"
    ctx.check(
        "jog dial marker moves around the wheel axis",
        marker_rotates_forward,
        details=detail,
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
