from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    CylinderGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="factory_wall_lamp", assets=ASSETS)

    steel = model.material("steel", rgba=(0.18, 0.19, 0.21, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.56, 0.58, 0.60, 1.0))
    enamel_green = model.material("enamel_green", rgba=(0.18, 0.28, 0.20, 1.0))
    reflector_white = model.material("reflector_white", rgba=(0.88, 0.88, 0.84, 1.0))

    def make_open_shade_shell(
        *,
        outer_radius: float,
        inner_radius: float,
        length: float,
        rear_wall: float,
        filename: str,
    ):
        outer = CylinderGeometry(radius=outer_radius, height=length, radial_segments=48, closed=True)
        relief = 0.004
        inner_length = length - rear_wall + (2.0 * relief)
        inner = CylinderGeometry(radius=inner_radius, height=inner_length, radial_segments=48, closed=True)
        inner.translate(0.0, 0.0, (rear_wall * 0.5) + relief)
        shell = boolean_difference(outer, inner)
        shell.rotate_y(math.pi / 2.0)
        return mesh_from_geometry(shell, ASSETS.mesh_path(filename))

    shade_shell_mesh = make_open_shade_shell(
        outer_radius=0.058,
        inner_radius=0.049,
        length=0.120,
        rear_wall=0.010,
        filename="lamp_shade_shell.obj",
    )
    reflector_liner_mesh = make_open_shade_shell(
        outer_radius=0.0455,
        inner_radius=0.0415,
        length=0.094,
        rear_wall=0.004,
        filename="lamp_reflector_liner.obj",
    )

    bracket = model.part("bracket")
    bracket.visual(
        Box((0.012, 0.095, 0.220)),
        origin=Origin(xyz=(-0.006, 0.0, 0.0)),
        material=steel,
        name="backplate",
    )
    bracket.visual(
        Box((0.010, 0.050, 0.096)),
        origin=Origin(xyz=(0.005, 0.0, 0.0)),
        material=steel,
        name="standoff",
    )
    bracket.visual(
        Cylinder(radius=0.0095, length=0.060),
        origin=Origin(xyz=(0.016, 0.0, 0.0)),
        material=satin_steel,
        name="shoulder_post",
    )
    bracket.visual(
        Cylinder(radius=0.012, length=0.008),
        origin=Origin(xyz=(-0.004, 0.0, 0.068), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="upper_anchor_cap",
    )
    bracket.visual(
        Cylinder(radius=0.012, length=0.008),
        origin=Origin(xyz=(-0.004, 0.0, -0.068), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="lower_anchor_cap",
    )
    bracket.visual(
        Box((0.016, 0.068, 0.020)),
        origin=Origin(xyz=(-0.002, 0.0, 0.066)),
        material=steel,
        name="upper_rib",
    )
    bracket.visual(
        Box((0.016, 0.068, 0.020)),
        origin=Origin(xyz=(-0.002, 0.0, -0.066)),
        material=steel,
        name="lower_rib",
    )

    boom = model.part("boom")
    boom.visual(
        Cylinder(radius=0.012, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=satin_steel,
        name="shoulder_sleeve",
    )
    boom.visual(
        Box((0.044, 0.020, 0.066)),
        origin=Origin(xyz=(0.022, 0.0, 0.0)),
        material=steel,
        name="shoulder_clevis",
    )
    boom.visual(
        Box((0.290, 0.014, 0.014)),
        origin=Origin(xyz=(0.173, 0.0, 0.026)),
        material=steel,
        name="upper_strut",
    )
    boom.visual(
        Box((0.290, 0.014, 0.014)),
        origin=Origin(xyz=(0.173, 0.0, -0.026)),
        material=steel,
        name="lower_strut",
    )
    boom.visual(
        Box((0.028, 0.022, 0.064)),
        origin=Origin(xyz=(0.310, 0.0, 0.0)),
        material=steel,
        name="head_brace",
    )
    boom.visual(
        Cylinder(radius=0.014, length=0.042),
        origin=Origin(xyz=(0.332, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="head_sleeve",
    )
    boom.visual(
        Cylinder(radius=0.0065, length=0.252),
        origin=Origin(xyz=(0.188, 0.0, 0.035), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="tension_rod",
    )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.011, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="trunnion",
    )
    head.visual(
        Box((0.050, 0.022, 0.022)),
        origin=Origin(xyz=(0.025, 0.0, 0.0)),
        material=steel,
        name="neck_stem",
    )
    head.visual(
        Cylinder(radius=0.024, length=0.028),
        origin=Origin(xyz=(0.055, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="collar",
    )
    head.visual(
        shade_shell_mesh,
        origin=Origin(xyz=(0.110, 0.0, -0.020), rpy=(0.0, 0.58, 0.0)),
        material=enamel_green,
        name="shade_shell",
    )
    head.visual(
        reflector_liner_mesh,
        origin=Origin(xyz=(0.109, 0.0, -0.020), rpy=(0.0, 0.58, 0.0)),
        material=reflector_white,
        name="reflector_liner",
    )
    head.visual(
        Cylinder(radius=0.061, length=0.006),
        origin=Origin(xyz=(0.160, 0.0, -0.054), rpy=(0.0, 0.58 + (math.pi / 2.0), 0.0)),
        material=satin_steel,
        name="front_rim",
    )

    model.articulation(
        "shoulder_swing",
        ArticulationType.REVOLUTE,
        parent=bracket,
        child=boom,
        origin=Origin(xyz=(0.016, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.8, lower=-1.05, upper=1.05),
    )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=boom,
        child=head,
        origin=Origin(xyz=(0.332, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=-0.28, upper=0.55),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    bracket = object_model.get_part("bracket")
    boom = object_model.get_part("boom")
    head = object_model.get_part("head")
    shoulder_swing = object_model.get_articulation("shoulder_swing")
    head_tilt = object_model.get_articulation("head_tilt")

    backplate = bracket.get_visual("backplate")
    shoulder_post = bracket.get_visual("shoulder_post")
    shoulder_sleeve = boom.get_visual("shoulder_sleeve")
    head_sleeve = boom.get_visual("head_sleeve")
    trunnion = head.get_visual("trunnion")
    neck_stem = head.get_visual("neck_stem")
    shade_shell = head.get_visual("shade_shell")
    reflector_liner = head.get_visual("reflector_liner")

    ctx.allow_overlap(boom, bracket, reason="shoulder sleeve rotates around the fixed wall post")
    ctx.allow_overlap(head, boom, reason="head trunnion nests inside the boom's tilt sleeve")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_within(
        bracket,
        boom,
        axes="xy",
        inner_elem=shoulder_post,
        outer_elem=shoulder_sleeve,
        name="wall_post_nested_inside_shoulder_sleeve",
    )
    ctx.expect_overlap(
        bracket,
        boom,
        axes="xy",
        min_overlap=0.015,
        elem_a=shoulder_post,
        elem_b=shoulder_sleeve,
        name="shoulder_pivot_has_full_bearing_overlap",
    )
    ctx.expect_within(
        head,
        boom,
        axes="xz",
        inner_elem=trunnion,
        outer_elem=head_sleeve,
        name="head_trunnion_nested_inside_tilt_sleeve",
    )
    ctx.expect_overlap(
        head,
        boom,
        axes="xz",
        min_overlap=0.020,
        elem_a=trunnion,
        elem_b=head_sleeve,
        name="tilt_pivot_has_full_bearing_overlap",
    )
    ctx.expect_gap(
        head,
        bracket,
        axis="x",
        min_gap=0.300,
        positive_elem=neck_stem,
        negative_elem=backplate,
        name="lamp_head_projects_out_from_wall_bracket",
    )
    ctx.expect_within(
        head,
        head,
        axes="yz",
        inner_elem=reflector_liner,
        outer_elem=shade_shell,
        name="reflector_nested_inside_deep_shade",
    )
    ctx.expect_overlap(
        head,
        head,
        axes="yz",
        min_overlap=0.070,
        elem_a=reflector_liner,
        elem_b=shade_shell,
        name="reflector_fills_shade_aperture",
    )
    with ctx.pose({head_tilt: 0.55}):
        ctx.expect_within(
            head,
            boom,
            axes="xz",
            inner_elem=trunnion,
            outer_elem=head_sleeve,
            name="head_stays_seated_when_tilted_down",
        )
    with ctx.pose({shoulder_swing: 1.00, head_tilt: 0.35}):
        ctx.expect_gap(
            head,
            bracket,
            axis="x",
            min_gap=0.140,
            positive_elem=shade_shell,
            negative_elem=backplate,
            name="swung_arm_keeps_shade_forward_of_wall",
        )
    with ctx.pose({shoulder_swing: -1.00, head_tilt: -0.20}):
        ctx.expect_gap(
            head,
            bracket,
            axis="x",
            min_gap=0.150,
            positive_elem=neck_stem,
            negative_elem=backplate,
            name="opposite_swing_keeps_head_out_from_bracket",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
