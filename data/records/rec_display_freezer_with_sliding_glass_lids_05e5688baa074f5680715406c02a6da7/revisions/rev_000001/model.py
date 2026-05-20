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
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="curved_lid_display_freezer")

    cabinet_white = model.material("cabinet_white", rgba=(0.90, 0.92, 0.95, 1.0))
    cabinet_dark = model.material("cabinet_dark", rgba=(0.20, 0.22, 0.24, 1.0))
    liner_white = model.material("liner_white", rgba=(0.97, 0.98, 0.99, 1.0))
    rail_aluminum = model.material("rail_aluminum", rgba=(0.72, 0.74, 0.78, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.70, 0.84, 0.92, 0.38))
    knob_black = model.material("knob_black", rgba=(0.12, 0.12, 0.13, 1.0))

    outer_length = 1.52
    outer_width = 0.86
    body_height = 0.74
    opening_length = 1.36
    opening_width = 0.70
    shoulder_radius = 0.07
    wall_base_height = 0.54
    floor_thickness = 0.06

    lower_track_top = body_height + 0.016
    upper_track_top = body_height + 0.050
    lower_track_y = 0.355
    upper_track_y = 0.319

    lower_lid_length = 0.80
    upper_lid_length = 0.80
    lower_lid_span = 0.73
    upper_lid_span = 0.64
    lower_rest_x = -0.29
    upper_rest_x = 0.29

    def curved_lid_mesh(
        *,
        length: float,
        span: float,
        rise: float,
        shell_thickness: float,
        edge_height: float,
        logical_name: str,
    ):
        half_span = span / 2.0
        x_samples = 5
        y_samples = 19
        xs = [-length / 2.0 + length * i / (x_samples - 1) for i in range(x_samples)]
        ys = [-half_span + span * j / (y_samples - 1) for j in range(y_samples)]

        def arch_height(y: float) -> float:
            norm = min(1.0, abs(y) / half_span) if half_span > 0.0 else 0.0
            return edge_height + rise * (1.0 - norm * norm)

        geom = MeshGeometry()
        outer: list[list[int]] = []
        inner: list[list[int]] = []

        for x_pos in xs:
            outer_row: list[int] = []
            inner_row: list[int] = []
            for y_pos in ys:
                z_outer = arch_height(y_pos)
                z_inner = max(0.002, z_outer - shell_thickness)
                outer_row.append(geom.add_vertex(x_pos, y_pos, z_outer))
                inner_row.append(geom.add_vertex(x_pos, y_pos, z_inner))
            outer.append(outer_row)
            inner.append(inner_row)

        def add_quad(a: int, b: int, c: int, d: int) -> None:
            geom.add_face(a, b, c)
            geom.add_face(a, c, d)

        for i in range(x_samples - 1):
            for j in range(y_samples - 1):
                add_quad(outer[i][j], outer[i + 1][j], outer[i + 1][j + 1], outer[i][j + 1])
                add_quad(inner[i][j + 1], inner[i + 1][j + 1], inner[i + 1][j], inner[i][j])

        for i in range(x_samples - 1):
            add_quad(outer[i][0], outer[i + 1][0], inner[i + 1][0], inner[i][0])
            add_quad(inner[i][-1], inner[i + 1][-1], outer[i + 1][-1], outer[i][-1])

        for j in range(y_samples - 1):
            add_quad(outer[0][j], outer[0][j + 1], inner[0][j + 1], inner[0][j])
            add_quad(inner[-1][j], inner[-1][j + 1], outer[-1][j + 1], outer[-1][j])

        return mesh_from_geometry(geom, logical_name)

    cabinet = model.part("cabinet")

    cabinet.visual(
        Box((1.40, 0.74, floor_thickness)),
        origin=Origin(xyz=(0.0, 0.0, floor_thickness / 2.0)),
        material=cabinet_dark,
        name="outer_floor",
    )
    cabinet.visual(
        Box((opening_length, 0.08, wall_base_height)),
        origin=Origin(xyz=(0.0, outer_width / 2.0 - 0.04, floor_thickness + wall_base_height / 2.0)),
        material=cabinet_white,
        name="front_wall",
    )
    cabinet.visual(
        Box((opening_length, 0.08, wall_base_height)),
        origin=Origin(xyz=(0.0, -outer_width / 2.0 + 0.04, floor_thickness + wall_base_height / 2.0)),
        material=cabinet_white,
        name="rear_wall",
    )
    cabinet.visual(
        Box((0.08, opening_width, wall_base_height)),
        origin=Origin(xyz=(outer_length / 2.0 - 0.04, 0.0, floor_thickness + wall_base_height / 2.0)),
        material=cabinet_white,
        name="right_end_wall",
    )
    cabinet.visual(
        Box((0.08, opening_width, wall_base_height)),
        origin=Origin(xyz=(-outer_length / 2.0 + 0.04, 0.0, floor_thickness + wall_base_height / 2.0)),
        material=cabinet_white,
        name="left_end_wall",
    )

    cabinet.visual(
        Cylinder(radius=shoulder_radius, length=opening_length),
        origin=Origin(
            xyz=(0.0, outer_width / 2.0 - shoulder_radius, body_height - shoulder_radius),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=cabinet_white,
        name="front_shoulder",
    )
    cabinet.visual(
        Cylinder(radius=shoulder_radius, length=opening_length),
        origin=Origin(
            xyz=(0.0, -outer_width / 2.0 + shoulder_radius, body_height - shoulder_radius),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=cabinet_white,
        name="rear_shoulder",
    )
    cabinet.visual(
        Cylinder(radius=shoulder_radius, length=opening_width),
        origin=Origin(
            xyz=(outer_length / 2.0 - shoulder_radius, 0.0, body_height - shoulder_radius),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=cabinet_white,
        name="right_shoulder",
    )
    cabinet.visual(
        Cylinder(radius=shoulder_radius, length=opening_width),
        origin=Origin(
            xyz=(-outer_length / 2.0 + shoulder_radius, 0.0, body_height - shoulder_radius),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=cabinet_white,
        name="left_shoulder",
    )

    cabinet.visual(
        Box((1.28, 0.60, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, floor_thickness + 0.0125)),
        material=liner_white,
        name="liner_floor",
    )
    cabinet.visual(
        Box((1.28, 0.03, 0.60)),
        origin=Origin(xyz=(0.0, 0.315, floor_thickness + 0.30)),
        material=liner_white,
        name="liner_front_wall",
    )
    cabinet.visual(
        Box((1.28, 0.03, 0.60)),
        origin=Origin(xyz=(0.0, -0.315, floor_thickness + 0.30)),
        material=liner_white,
        name="liner_rear_wall",
    )
    cabinet.visual(
        Box((0.03, 0.60, 0.60)),
        origin=Origin(xyz=(0.625, 0.0, floor_thickness + 0.30)),
        material=liner_white,
        name="liner_right_wall",
    )
    cabinet.visual(
        Box((0.03, 0.60, 0.60)),
        origin=Origin(xyz=(-0.625, 0.0, floor_thickness + 0.30)),
        material=liner_white,
        name="liner_left_wall",
    )

    cabinet.visual(
        Box((opening_length, 0.04, 0.016)),
        origin=Origin(xyz=(0.0, lower_track_y, lower_track_top - 0.008)),
        material=rail_aluminum,
        name="lower_front_track",
    )
    cabinet.visual(
        Box((opening_length, 0.04, 0.016)),
        origin=Origin(xyz=(0.0, -lower_track_y, lower_track_top - 0.008)),
        material=rail_aluminum,
        name="lower_rear_track",
    )
    cabinet.visual(
        Box((opening_length, 0.032, 0.016)),
        origin=Origin(xyz=(0.0, upper_track_y, upper_track_top - 0.008)),
        material=rail_aluminum,
        name="upper_front_track",
    )
    cabinet.visual(
        Box((opening_length, 0.032, 0.016)),
        origin=Origin(xyz=(0.0, -upper_track_y, upper_track_top - 0.008)),
        material=rail_aluminum,
        name="upper_rear_track",
    )
    cabinet.visual(
        Box((opening_length, 0.004, upper_track_top - lower_track_top + 0.002)),
        origin=Origin(
            xyz=(
                0.0,
                0.335,
                (upper_track_top + lower_track_top) / 2.0 + 0.001,
            )
        ),
        material=rail_aluminum,
        name="front_track_bridge",
    )
    cabinet.visual(
        Box((opening_length, 0.004, upper_track_top - lower_track_top + 0.002)),
        origin=Origin(
            xyz=(
                0.0,
                -0.335,
                (upper_track_top + lower_track_top) / 2.0 + 0.001,
            )
        ),
        material=rail_aluminum,
        name="rear_track_bridge",
    )

    cabinet.visual(
        Box((0.11, 0.035, 0.08)),
        origin=Origin(xyz=(0.50, outer_width / 2.0 + 0.0175, 0.34)),
        material=cabinet_dark,
        name="control_pod",
    )

    cabinet.inertial = Inertial.from_geometry(
        Box((outer_length, outer_width, body_height + 0.08)),
        mass=78.0,
        origin=Origin(xyz=(0.0, 0.0, (body_height + 0.08) / 2.0)),
    )

    lower_lid = model.part("lower_lid")
    lower_lid_rib_depth = 0.024
    lower_lid.visual(
        curved_lid_mesh(
            length=lower_lid_length,
            span=0.56,
            rise=0.090,
            shell_thickness=0.005,
            edge_height=0.016,
            logical_name="lower_glass_lid",
        ),
        material=glass_tint,
        name="lower_lid_shell",
    )
    lower_lid.visual(
        Box((lower_lid_length, 0.090, 0.016)),
        origin=Origin(xyz=(0.0, 0.322, 0.008)),
        material=rail_aluminum,
        name="lower_front_side_frame",
    )
    lower_lid.visual(
        Box((lower_lid_length, 0.090, 0.016)),
        origin=Origin(xyz=(0.0, -0.322, 0.008)),
        material=rail_aluminum,
        name="lower_rear_side_frame",
    )
    lower_lid.visual(
        curved_lid_mesh(
            length=lower_lid_rib_depth,
            span=0.56,
            rise=0.090,
            shell_thickness=0.010,
            edge_height=0.020,
            logical_name="lower_right_end_rib",
        ),
        origin=Origin(xyz=(lower_lid_length / 2.0 - lower_lid_rib_depth / 2.0, 0.0, 0.0)),
        material=rail_aluminum,
        name="lower_right_end_frame",
    )
    lower_lid.visual(
        curved_lid_mesh(
            length=lower_lid_rib_depth,
            span=0.56,
            rise=0.090,
            shell_thickness=0.010,
            edge_height=0.020,
            logical_name="lower_left_end_rib",
        ),
        origin=Origin(xyz=(-lower_lid_length / 2.0 + lower_lid_rib_depth / 2.0, 0.0, 0.0)),
        material=rail_aluminum,
        name="lower_left_end_frame",
    )
    lower_lid.visual(
        Box((lower_lid_length, 0.026, 0.018)),
        origin=Origin(xyz=(0.0, lower_track_y, 0.009)),
        material=rail_aluminum,
        name="lower_front_runner",
    )
    lower_lid.visual(
        Box((lower_lid_length, 0.026, 0.018)),
        origin=Origin(xyz=(0.0, -lower_track_y, 0.009)),
        material=rail_aluminum,
        name="lower_rear_runner",
    )
    lower_lid.inertial = Inertial.from_geometry(
        Box((lower_lid_length, lower_lid_span, 0.14)),
        mass=6.5,
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
    )

    upper_lid = model.part("upper_lid")
    upper_lid_rib_depth = 0.024
    upper_lid_frame_span = 0.64
    upper_lid.visual(
        curved_lid_mesh(
            length=upper_lid_length,
            span=0.48,
            rise=0.082,
            shell_thickness=0.005,
            edge_height=0.016,
            logical_name="upper_glass_lid",
        ),
        material=glass_tint,
        name="upper_lid_shell",
    )
    upper_lid.visual(
        Box((upper_lid_length, 0.040, 0.014)),
        origin=Origin(xyz=(0.0, 0.300, 0.011)),
        material=rail_aluminum,
        name="upper_front_side_frame",
    )
    upper_lid.visual(
        Box((upper_lid_length, 0.040, 0.014)),
        origin=Origin(xyz=(0.0, -0.300, 0.011)),
        material=rail_aluminum,
        name="upper_rear_side_frame",
    )
    upper_lid.visual(
        curved_lid_mesh(
            length=upper_lid_rib_depth,
            span=upper_lid_frame_span,
            rise=0.082,
            shell_thickness=0.008,
            edge_height=0.022,
            logical_name="upper_right_end_rib",
        ),
        origin=Origin(xyz=(upper_lid_length / 2.0 - upper_lid_rib_depth / 2.0, 0.0, 0.0)),
        material=rail_aluminum,
        name="upper_right_end_frame",
    )
    upper_lid.visual(
        curved_lid_mesh(
            length=upper_lid_rib_depth,
            span=upper_lid_frame_span,
            rise=0.082,
            shell_thickness=0.008,
            edge_height=0.022,
            logical_name="upper_left_end_rib",
        ),
        origin=Origin(xyz=(-upper_lid_length / 2.0 + upper_lid_rib_depth / 2.0, 0.0, 0.0)),
        material=rail_aluminum,
        name="upper_left_end_frame",
    )
    upper_lid.visual(
        Box((upper_lid_length, 0.028, 0.018)),
        origin=Origin(xyz=(0.0, upper_track_y, 0.009)),
        material=rail_aluminum,
        name="upper_front_runner",
    )
    upper_lid.visual(
        Box((upper_lid_length, 0.028, 0.018)),
        origin=Origin(xyz=(0.0, -upper_track_y, 0.009)),
        material=rail_aluminum,
        name="upper_rear_runner",
    )
    upper_lid.inertial = Inertial.from_geometry(
        Box((upper_lid_length, upper_lid_span, 0.13)),
        mass=5.8,
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
    )

    knob = model.part("control_knob")
    knob.visual(
        Cylinder(radius=0.008, length=0.022),
        origin=Origin(xyz=(0.0, 0.011, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=knob_black,
        name="knob_shaft",
    )
    knob.visual(
        Cylinder(radius=0.028, length=0.018),
        origin=Origin(xyz=(0.0, 0.031, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=knob_black,
        name="knob_dial",
    )
    knob.visual(
        Box((0.010, 0.008, 0.004)),
        origin=Origin(xyz=(0.0, 0.040, 0.020)),
        material=rail_aluminum,
        name="knob_pointer",
    )
    knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.03, length=0.04),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.02, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "cabinet_to_lower_lid",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=lower_lid,
        origin=Origin(xyz=(lower_rest_x, 0.0, lower_track_top)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.45,
            lower=0.0,
            upper=0.44,
        ),
    )
    model.articulation(
        "cabinet_to_upper_lid",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=upper_lid,
        origin=Origin(xyz=(upper_rest_x, 0.0, upper_track_top)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.45,
            lower=0.0,
            upper=0.44,
        ),
    )
    model.articulation(
        "cabinet_to_knob",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=knob,
        origin=Origin(xyz=(0.50, outer_width / 2.0 + 0.035, 0.34)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.5,
            lower=-math.radians(140.0),
            upper=math.radians(140.0),
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

    cabinet = object_model.get_part("cabinet")
    lower_lid = object_model.get_part("lower_lid")
    upper_lid = object_model.get_part("upper_lid")
    knob = object_model.get_part("control_knob")

    lower_slide = object_model.get_articulation("cabinet_to_lower_lid")
    upper_slide = object_model.get_articulation("cabinet_to_upper_lid")
    knob_joint = object_model.get_articulation("cabinet_to_knob")

    ctx.check("cabinet part exists", cabinet is not None)
    ctx.check("lower lid part exists", lower_lid is not None)
    ctx.check("upper lid part exists", upper_lid is not None)
    ctx.check("control knob part exists", knob is not None)

    ctx.check(
        "lower lid slides toward +x",
        tuple(round(v, 3) for v in lower_slide.axis) == (1.0, 0.0, 0.0),
        details=f"axis={lower_slide.axis}",
    )
    ctx.check(
        "upper lid slides toward -x",
        tuple(round(v, 3) for v in upper_slide.axis) == (-1.0, 0.0, 0.0),
        details=f"axis={upper_slide.axis}",
    )
    ctx.check(
        "control knob rotates about front-facing shaft",
        tuple(round(v, 3) for v in knob_joint.axis) == (0.0, 1.0, 0.0),
        details=f"axis={knob_joint.axis}",
    )

    ctx.expect_contact(
        lower_lid,
        cabinet,
        contact_tol=0.001,
        name="lower lid sits on the lower guide rails",
    )
    ctx.expect_contact(
        upper_lid,
        cabinet,
        contact_tol=0.001,
        name="upper lid sits on the upper guide rails",
    )
    ctx.expect_contact(
        knob,
        cabinet,
        contact_tol=0.001,
        name="control knob mounts against the front control pod",
    )
    ctx.expect_overlap(
        lower_lid,
        upper_lid,
        axes="xy",
        min_overlap=0.18,
        name="curved lids overlap across the center opening at rest",
    )
    lower_rest = ctx.part_world_position(lower_lid)
    upper_rest = ctx.part_world_position(upper_lid)
    knob_rest = ctx.part_world_position(knob)
    ctx.check(
        "upper lid rides on a higher rail set",
        lower_rest is not None and upper_rest is not None and upper_rest[2] > lower_rest[2] + 0.02,
        details=f"lower_rest={lower_rest}, upper_rest={upper_rest}",
    )

    lower_upper_limit = lower_slide.motion_limits.upper if lower_slide.motion_limits else None
    upper_upper_limit = upper_slide.motion_limits.upper if upper_slide.motion_limits else None

    if lower_upper_limit is not None and upper_upper_limit is not None:
        with ctx.pose({lower_slide: lower_upper_limit, upper_slide: upper_upper_limit}):
            ctx.expect_contact(
                lower_lid,
                cabinet,
                contact_tol=0.001,
                name="lower lid remains guided when opened",
            )
            ctx.expect_contact(
                upper_lid,
                cabinet,
                contact_tol=0.001,
                name="upper lid remains guided when opened",
            )
            lower_open = ctx.part_world_position(lower_lid)
            upper_open = ctx.part_world_position(upper_lid)

        ctx.check(
            "lower lid opens by sliding rightward",
            lower_rest is not None
            and lower_open is not None
            and lower_open[0] > lower_rest[0] + 0.20
            and abs(lower_open[1] - lower_rest[1]) < 1e-6
            and abs(lower_open[2] - lower_rest[2]) < 1e-6,
            details=f"rest={lower_rest}, open={lower_open}",
        )
        ctx.check(
            "upper lid opens by sliding leftward",
            upper_rest is not None
            and upper_open is not None
            and upper_open[0] < upper_rest[0] - 0.20
            and abs(upper_open[1] - upper_rest[1]) < 1e-6
            and abs(upper_open[2] - upper_rest[2]) < 1e-6,
            details=f"rest={upper_rest}, open={upper_open}",
        )
        ctx.check(
            "opened lids keep their stepped rail heights",
            lower_open is not None and upper_open is not None and upper_open[2] > lower_open[2] + 0.02,
            details=f"lower_open={lower_open}, upper_open={upper_open}",
        )

    with ctx.pose({knob_joint: 1.1}):
        knob_turned = ctx.part_world_position(knob)

    ctx.check(
        "control knob turns in place",
        knob_rest is not None
        and knob_turned is not None
        and max(abs(a - b) for a, b in zip(knob_rest, knob_turned)) < 1e-6,
        details=f"rest={knob_rest}, turned={knob_turned}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
