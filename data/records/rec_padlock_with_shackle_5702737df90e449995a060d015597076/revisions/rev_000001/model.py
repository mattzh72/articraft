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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    wire_from_points,
)


def _build_body_lower_mesh(
    *,
    width: float,
    depth: float,
    height: float,
    corner_radius: float,
):
    geom = ExtrudeGeometry.centered(
        rounded_rect_profile(width, height, corner_radius, corner_segments=8),
        depth,
        cap=True,
        closed=True,
    )
    geom.rotate_x(math.pi / 2.0)
    geom.translate(0.0, 0.0, height * 0.5)
    return mesh_from_geometry(geom, "padlock_body_lower")


def _build_shackle_mesh(
    *,
    span: float,
    crown_height: float,
    rod_radius: float,
    retained_drop: float,
    free_drop: float,
):
    path = [
        (0.0, 0.0, -retained_drop),
        (0.0, 0.0, crown_height),
        (span, 0.0, crown_height),
        (span, 0.0, free_drop),
    ]
    geom = wire_from_points(
        path,
        radius=rod_radius,
        closed_path=False,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.0045,
        corner_segments=16,
        radial_segments=18,
    )
    return mesh_from_geometry(geom, "padlock_shackle")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="combination_padlock")

    body_finish = model.material("body_finish", rgba=(0.18, 0.19, 0.21, 1.0))
    body_panel = model.material("body_panel", rgba=(0.28, 0.29, 0.31, 1.0))
    shackle_finish = model.material("shackle_finish", rgba=(0.86, 0.88, 0.90, 1.0))
    dial_finish = model.material("dial_finish", rgba=(0.10, 0.11, 0.12, 1.0))
    marker_finish = model.material("marker_finish", rgba=(0.80, 0.81, 0.83, 1.0))

    body_width = 0.046
    body_depth = 0.023
    lower_body_height = 0.050
    top_cap_height = 0.010
    total_body_height = lower_body_height + top_cap_height
    shackle_left_x = -0.012
    shackle_right_x = 0.012
    shackle_span = shackle_right_x - shackle_left_x

    body = model.part("body")
    body.visual(
        _build_body_lower_mesh(
            width=body_width,
            depth=body_depth,
            height=lower_body_height,
            corner_radius=0.0055,
        ),
        material=body_finish,
        name="body_shell",
    )

    top_block_z = lower_body_height + top_cap_height * 0.5
    top_depth_band = 0.0092
    top_strip_depth = 0.0074
    hole_half_width = 0.0034
    center_block_width = 2.0 * (shackle_right_x - hole_half_width) + 0.0006
    outer_block_width = ((body_width * 0.5) - (shackle_right_x + hole_half_width)) + 0.0008

    body.visual(
        Box((body_width, top_strip_depth, top_cap_height)),
        origin=Origin(xyz=(0.0, (body_depth - top_strip_depth) * 0.5, top_block_z)),
        material=body_finish,
        name="top_front_strip",
    )
    body.visual(
        Box((body_width, top_strip_depth, top_cap_height)),
        origin=Origin(xyz=(0.0, -(body_depth - top_strip_depth) * 0.5, top_block_z)),
        material=body_finish,
        name="top_back_strip",
    )
    body.visual(
        Box((outer_block_width, top_depth_band, top_cap_height)),
        origin=Origin(
            xyz=(
                -body_width * 0.5 + outer_block_width * 0.5,
                0.0,
                top_block_z,
            )
        ),
        material=body_finish,
        name="top_left_block",
    )
    body.visual(
        Box((center_block_width, top_depth_band, top_cap_height)),
        origin=Origin(xyz=(0.0, 0.0, top_block_z)),
        material=body_finish,
        name="top_center_block",
    )
    body.visual(
        Box((outer_block_width, top_depth_band, top_cap_height)),
        origin=Origin(
            xyz=(
                body_width * 0.5 - outer_block_width * 0.5,
                0.0,
                top_block_z,
            )
        ),
        material=body_finish,
        name="top_right_block",
    )
    body.visual(
        Box((0.0006, 0.0052, top_cap_height)),
        origin=Origin(
            xyz=(
                shackle_left_x - 0.0027,
                0.0,
                top_block_z,
            )
        ),
        material=body_finish,
        name="retained_leg_keeper",
    )

    face_panel_thickness = 0.0022
    body.visual(
        Box((0.039, face_panel_thickness, 0.020)),
        origin=Origin(
            xyz=(
                0.0,
                body_depth * 0.5 + face_panel_thickness * 0.5 - 0.0002,
                0.030,
            )
        ),
        material=body_panel,
        name="dial_panel",
    )
    body.visual(
        Box((0.041, 0.0030, 0.0040)),
        origin=Origin(
            xyz=(
                0.0,
                body_depth * 0.5 + 0.0012,
                0.041,
            )
        ),
        material=body_panel,
        name="upper_face_lip",
    )
    body.visual(
        Box((0.041, 0.0030, 0.0040)),
        origin=Origin(
            xyz=(
                0.0,
                body_depth * 0.5 + 0.0012,
                0.019,
            )
        ),
        material=body_panel,
        name="lower_face_lip",
    )
    body.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, total_body_height)),
        mass=0.36,
        origin=Origin(xyz=(0.0, 0.0, total_body_height * 0.5)),
    )

    shackle = model.part("shackle")
    shackle.visual(
        _build_shackle_mesh(
            span=shackle_span,
            crown_height=0.030,
            rod_radius=0.0024,
            retained_drop=0.006,
            free_drop=-0.0045,
        ),
        material=shackle_finish,
        name="shackle_bar",
    )
    shackle.visual(
        Sphere(radius=0.0012),
        origin=Origin(xyz=(shackle_span, 0.0, -0.0045)),
        material=shackle_finish,
        name="free_leg_tip",
    )
    shackle.inertial = Inertial.from_geometry(
        Box((0.029, 0.005, 0.041)),
        mass=0.05,
        origin=Origin(xyz=(0.012, 0.0, 0.014)),
    )

    shackle_joint = model.articulation(
        "body_to_shackle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=shackle,
        origin=Origin(xyz=(shackle_left_x, 0.0, total_body_height)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.35,
            velocity=2.0,
            lower=0.0,
            upper=1.6,
        ),
    )

    dial_radius = 0.0055
    dial_length = 0.0068
    dial_centers_x = (-0.012, -0.004, 0.004, 0.012)
    dial_center_y = body_depth * 0.5 + face_panel_thickness - 0.0002 + dial_radius
    dial_center_z = 0.030

    for index, center_x in enumerate(dial_centers_x, start=1):
        dial = model.part(f"dial_{index}")
        dial.visual(
            Cylinder(radius=dial_radius, length=dial_length),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dial_finish,
            name="dial_wheel",
        )
        dial.visual(
            Box((dial_length * 0.68, 0.0012, 0.0016)),
            origin=Origin(xyz=(0.0, 0.0, dial_radius - 0.0005)),
            material=marker_finish,
            name="digit_marker",
        )
        dial.inertial = Inertial.from_geometry(
            Cylinder(radius=dial_radius, length=dial_length),
            mass=0.008,
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        )
        model.articulation(
            f"body_to_dial_{index}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=dial,
            origin=Origin(xyz=(center_x, dial_center_y, dial_center_z)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.03, velocity=10.0),
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
    shackle = object_model.get_part("shackle")
    dial_1 = object_model.get_part("dial_1")
    dial_2 = object_model.get_part("dial_2")
    dial_3 = object_model.get_part("dial_3")
    dial_4 = object_model.get_part("dial_4")
    shackle_joint = object_model.get_articulation("body_to_shackle")
    dial_1_joint = object_model.get_articulation("body_to_dial_1")

    for part_name in ("body", "shackle", "dial_1", "dial_2", "dial_3", "dial_4"):
        ctx.check(
            f"{part_name} present",
            object_model.get_part(part_name) is not None,
            details="",
        )

    ctx.expect_gap(
        dial_1,
        body,
        axis="y",
        min_gap=0.0,
        max_gap=0.0015,
        negative_elem="dial_panel",
        name="left dial sits on the front face",
    )
    ctx.expect_gap(
        dial_4,
        body,
        axis="y",
        min_gap=0.0,
        max_gap=0.0015,
        negative_elem="dial_panel",
        name="right dial sits on the front face",
    )
    ctx.expect_overlap(
        dial_1,
        body,
        axes="z",
        min_overlap=0.010,
        name="left dial aligns with body height band",
    )
    ctx.expect_overlap(
        shackle,
        body,
        axes="x",
        min_overlap=0.020,
        name="closed shackle spans the body width",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    closed_free_tip = _aabb_center(ctx.part_element_world_aabb(shackle, elem="free_leg_tip"))
    with ctx.pose({shackle_joint: 1.4}):
        open_free_tip = _aabb_center(ctx.part_element_world_aabb(shackle, elem="free_leg_tip"))
    ctx.check(
        "shackle lifts upward on the retained leg",
        closed_free_tip is not None
        and open_free_tip is not None
        and open_free_tip[2] > closed_free_tip[2] + 0.015,
        details=f"closed={closed_free_tip}, open={open_free_tip}",
    )

    dial_1_marker_rest = _aabb_center(ctx.part_element_world_aabb(dial_1, elem="digit_marker"))
    dial_2_marker_rest = _aabb_center(ctx.part_element_world_aabb(dial_2, elem="digit_marker"))
    with ctx.pose({dial_1_joint: math.pi / 2.0}):
        dial_1_marker_turned = _aabb_center(ctx.part_element_world_aabb(dial_1, elem="digit_marker"))
        dial_2_marker_turned = _aabb_center(ctx.part_element_world_aabb(dial_2, elem="digit_marker"))

    ctx.check(
        "first dial rotates around its own short axis",
        dial_1_marker_rest is not None
        and dial_1_marker_turned is not None
        and dial_1_marker_turned[1] > dial_1_marker_rest[1] + 0.003
        and dial_1_marker_turned[2] < dial_1_marker_rest[2] - 0.003,
        details=f"rest={dial_1_marker_rest}, turned={dial_1_marker_turned}",
    )
    ctx.check(
        "dial articulations are independent",
        dial_2_marker_rest is not None
        and dial_2_marker_turned is not None
        and abs(dial_2_marker_turned[1] - dial_2_marker_rest[1]) < 1e-6
        and abs(dial_2_marker_turned[2] - dial_2_marker_rest[2]) < 1e-6,
        details=f"rest={dial_2_marker_rest}, turned={dial_2_marker_turned}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
