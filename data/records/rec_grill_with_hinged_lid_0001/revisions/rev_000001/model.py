from __future__ import annotations

# The harness only exposes the editable block to the model.
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
    Inertial,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_side_loft,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _make_body_mesh():
    return mesh_from_geometry(
        superellipse_side_loft(
            [
                (-0.22, 0.0, 0.095, 0.73),
                (-0.10, 0.0, 0.120, 0.76),
                (0.00, 0.0, 0.132, 0.78),
                (0.10, 0.0, 0.120, 0.76),
                (0.22, 0.0, 0.095, 0.73),
            ],
            exponents=3.1,
            segments=64,
        ),
        ASSETS.mesh_path("grill_firebox.obj"),
    )


def _make_lid_mesh():
    return mesh_from_geometry(
        superellipse_side_loft(
            [
                (-0.36, 0.0, 0.175, 0.66),
                (-0.26, 0.0, 0.215, 0.70),
                (-0.16, 0.0, 0.242, 0.72),
                (-0.06, 0.0, 0.240, 0.72),
                (0.04, 0.0, 0.170, 0.68),
            ],
            exponents=3.4,
            segments=64,
        ),
        ASSETS.mesh_path("grill_lid.obj"),
    )


def _build_knob(
    model: ArticulatedObject,
    name: str,
    x_pos: float,
    z_pos: float,
    knob_material: Material,
    trim_material: Material,
) -> None:
    knob = model.part(name)
    knob.visual(
        Cylinder(radius=0.035, length=0.048),
        origin=Origin(xyz=(0.0, -0.025, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_material,
    )
    knob.visual(
        Cylinder(radius=0.028, length=0.010),
        origin=Origin(xyz=(0.0, -0.055, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_material,
    )
    knob.visual(
        Box((0.009, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, -0.058, 0.041)),
        material=trim_material,
    )
    knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.038, length=0.060),
        mass=0.18,
        origin=Origin(xyz=(0.0, -0.030, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    model.articulation(
        f"{name}_joint",
        ArticulationType.REVOLUTE,
        parent="cart_body",
        child=name,
        origin=Origin(xyz=(x_pos, -0.339, z_pos)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=6.0,
            lower=0.0,
            upper=4.71238898038469,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="outdoor_grill", assets=ASSETS)

    painted_black = Material(name="painted_black", rgba=(0.15, 0.16, 0.17, 1.0))
    heat_enamel = Material(name="heat_enamel", rgba=(0.23, 0.24, 0.25, 1.0))
    stainless = Material(name="stainless", rgba=(0.74, 0.76, 0.78, 1.0))
    dark_plastic = Material(name="dark_plastic", rgba=(0.09, 0.09, 0.10, 1.0))
    rubber = Material(name="rubber", rgba=(0.07, 0.07, 0.08, 1.0))
    glass = Material(name="smoked_glass", rgba=(0.22, 0.25, 0.28, 0.45))
    model.materials.extend([painted_black, heat_enamel, stainless, dark_plastic, rubber, glass])

    cart_body = model.part("cart_body")
    cart_body.visual(
        Box((0.74, 0.52, 0.45)),
        origin=Origin(xyz=(0.0, 0.0, 0.345)),
        material=painted_black,
    )
    cart_body.visual(
        Box((0.35, 0.024, 0.39)),
        origin=Origin(xyz=(-0.185, -0.271, 0.335)),
        material=painted_black,
    )
    cart_body.visual(
        Box((0.35, 0.024, 0.39)),
        origin=Origin(xyz=(0.185, -0.271, 0.335)),
        material=painted_black,
    )
    cart_body.visual(
        Cylinder(radius=0.009, length=0.14),
        origin=Origin(xyz=(-0.11, -0.287, 0.35)),
        material=stainless,
    )
    cart_body.visual(
        Cylinder(radius=0.009, length=0.14),
        origin=Origin(xyz=(0.11, -0.287, 0.35)),
        material=stainless,
    )
    cart_body.visual(
        Box((0.68, 0.34, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.575)),
        material=heat_enamel,
    )
    cart_body.visual(
        Box((0.05, 0.40, 0.10)),
        origin=Origin(xyz=(-0.335, 0.0, 0.585)),
        material=heat_enamel,
    )
    cart_body.visual(
        Box((0.05, 0.40, 0.10)),
        origin=Origin(xyz=(0.335, 0.0, 0.585)),
        material=heat_enamel,
    )
    cart_body.visual(
        Box((0.62, 0.05, 0.10)),
        origin=Origin(xyz=(0.0, 0.185, 0.585)),
        material=heat_enamel,
    )
    cart_body.visual(
        Box((0.62, 0.04, 0.07)),
        origin=Origin(xyz=(0.0, -0.190, 0.575)),
        material=heat_enamel,
    )
    cart_body.visual(
        Box((0.74, 0.09, 0.11)),
        origin=Origin(xyz=(0.0, -0.295, 0.605)),
        material=stainless,
    )
    cart_body.visual(
        Box((0.66, 0.08, 0.03)),
        origin=Origin(xyz=(0.0, -0.214, 0.555)),
        material=heat_enamel,
    )
    cart_body.visual(
        Box((0.27, 0.44, 0.03)),
        origin=Origin(xyz=(-0.492, 0.0, 0.672)),
        material=stainless,
    )
    cart_body.visual(
        Box((0.27, 0.44, 0.03)),
        origin=Origin(xyz=(0.492, 0.0, 0.672)),
        material=stainless,
    )
    cart_body.visual(
        Box((0.08, 0.08, 0.06)),
        origin=Origin(xyz=(-0.355, 0.0, 0.642)),
        material=painted_black,
    )
    cart_body.visual(
        Box((0.08, 0.08, 0.06)),
        origin=Origin(xyz=(0.355, 0.0, 0.642)),
        material=painted_black,
    )
    cart_body.visual(
        Box((0.72, 0.032, 0.028)),
        origin=Origin(xyz=(0.0, 0.216, 0.647)),
        material=heat_enamel,
    )
    cart_body.visual(
        Box((0.66, 0.012, 0.012)),
        origin=Origin(xyz=(0.0, -0.105, 0.635)),
        material=stainless,
    )
    cart_body.visual(
        Box((0.66, 0.012, 0.012)),
        origin=Origin(xyz=(0.0, -0.035, 0.635)),
        material=stainless,
    )
    cart_body.visual(
        Box((0.66, 0.012, 0.012)),
        origin=Origin(xyz=(0.0, 0.035, 0.635)),
        material=stainless,
    )
    cart_body.visual(
        Box((0.66, 0.012, 0.012)),
        origin=Origin(xyz=(0.0, 0.105, 0.635)),
        material=stainless,
    )
    cart_body.visual(
        Box((0.33, 0.40, 0.018)),
        origin=Origin(xyz=(-0.19, -0.252, 0.345)),
        material=painted_black,
    )
    cart_body.visual(
        Box((0.33, 0.40, 0.018)),
        origin=Origin(xyz=(0.19, -0.252, 0.345)),
        material=painted_black,
    )
    cart_body.visual(
        Box((0.022, 0.16, 0.018)),
        origin=Origin(xyz=(-0.19, -0.282, 0.345)),
        material=stainless,
    )
    cart_body.visual(
        Box((0.022, 0.16, 0.018)),
        origin=Origin(xyz=(0.19, -0.282, 0.345)),
        material=stainless,
    )
    cart_body.visual(
        Cylinder(radius=0.018, length=0.82),
        origin=Origin(xyz=(0.0, 0.225, 0.122), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
    )
    cart_body.visual(
        Cylinder(radius=0.10, length=0.055),
        origin=Origin(xyz=(-0.415, 0.225, 0.122), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
    )
    cart_body.visual(
        Cylinder(radius=0.10, length=0.055),
        origin=Origin(xyz=(0.415, 0.225, 0.122), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
    )
    cart_body.visual(
        Box((0.06, 0.10, 0.12)),
        origin=Origin(xyz=(-0.28, -0.18, 0.06)),
        material=painted_black,
    )
    cart_body.visual(
        Box((0.06, 0.10, 0.12)),
        origin=Origin(xyz=(0.28, -0.18, 0.06)),
        material=painted_black,
    )
    cart_body.inertial = Inertial.from_geometry(
        Box((1.26, 0.60, 0.73)),
        mass=40.0,
        origin=Origin(xyz=(0.0, 0.0, 0.365)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((0.72, 0.03, 0.05)),
        origin=Origin(xyz=(0.0, -0.015, 0.030)),
        material=heat_enamel,
    )
    lid.visual(
        Box((0.72, 0.42, 0.14)),
        origin=Origin(xyz=(0.0, -0.210, 0.080)),
        material=painted_black,
    )
    lid.visual(
        Box((0.62, 0.30, 0.08)),
        origin=Origin(xyz=(0.0, -0.205, 0.190)),
        material=heat_enamel,
    )
    lid.visual(
        Box((0.03, 0.36, 0.14)),
        origin=Origin(xyz=(-0.345, -0.195, 0.080)),
        material=painted_black,
    )
    lid.visual(
        Box((0.03, 0.36, 0.14)),
        origin=Origin(xyz=(0.345, -0.195, 0.080)),
        material=painted_black,
    )
    lid.visual(
        Box((0.60, 0.03, 0.04)),
        origin=Origin(xyz=(0.0, -0.415, 0.055)),
        material=stainless,
    )
    lid.visual(
        Box((0.03, 0.05, 0.05)),
        origin=Origin(xyz=(-0.22, -0.365, 0.090)),
        material=stainless,
    )
    lid.visual(
        Box((0.03, 0.05, 0.05)),
        origin=Origin(xyz=(0.22, -0.365, 0.090)),
        material=stainless,
    )
    lid.visual(
        Cylinder(radius=0.012, length=0.44),
        origin=Origin(xyz=(0.0, -0.340, 0.100), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
    )
    lid.visual(
        Cylinder(radius=0.018, length=0.014),
        origin=Origin(xyz=(0.0, -0.175, 0.248), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
    )
    lid.visual(
        Cylinder(radius=0.014, length=0.006),
        origin=Origin(xyz=(0.0, -0.185, 0.248), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=glass,
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.74, 0.44, 0.26)),
        mass=12.0,
        origin=Origin(xyz=(0.0, -0.215, 0.115)),
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent="cart_body",
        child="lid",
        origin=Origin(xyz=(0.0, 0.205, 0.645)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=2.0,
            lower=0.0,
            upper=1.65,
        ),
    )

    knob_positions = (-0.24, -0.08, 0.08, 0.24)
    for index, x_pos in enumerate(knob_positions, start=1):
        _build_knob(
            model,
            name=f"knob_{index}",
            x_pos=x_pos,
            z_pos=0.605,
            knob_material=dark_plastic,
            trim_material=stainless,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.allow_overlap(
        "cart_body",
        "lid",
        reason="The grill hood is visually modeled as a solid shell over the cookbox, so generated collision hulls conservatively overlap through the enclosed cooking volume.",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=192, overlap_tol=0.004, overlap_volume_tol=0.0)

    ctx.expect_aabb_overlap("lid", "cart_body", axes="xy", min_overlap=0.24)

    def require(condition: bool, message: str) -> None:
        if not condition:
            raise AssertionError(message)

    class _AABBView:
        def __init__(self, min_corner, max_corner) -> None:
            self.min_x, self.min_y, self.min_z = min_corner
            self.max_x, self.max_y, self.max_z = max_corner

    def part_aabb(name: str):
        raw = ctx.part_world_aabb(name)
        if hasattr(raw, "max_x"):
            return raw
        if len(raw) == 2 and len(raw[0]) == 3 and len(raw[1]) == 3:
            return _AABBView(raw[0], raw[1])
        if len(raw) == 6:
            return _AABBView(raw[:3], raw[3:])
        raise AssertionError(f"Unsupported AABB format for {name}: {raw!r}")

    body_aabb = part_aabb("cart_body")
    require(
        body_aabb.max_x - body_aabb.min_x > 1.20, "The grill should include broad side shelves."
    )
    require(
        body_aabb.max_z - body_aabb.min_z > 0.68,
        "The grill body should read as a tall outdoor appliance.",
    )

    lid_closed = part_aabb("lid")
    require(
        lid_closed.max_z > body_aabb.max_z + 0.20,
        "The hood should rise clearly above the cookbox and control area.",
    )
    skirt_engagement = body_aabb.max_z - lid_closed.min_z
    require(
        0.02 < skirt_engagement < 0.08,
        "The lid skirt should nest over the cookbox rim without dropping unrealistically deep.",
    )

    with ctx.pose(lid_hinge=1.50):
        lid_open = part_aabb("lid")
        require(
            lid_open.max_z > lid_closed.max_z + 0.10, "Opening the lid should lift the hood higher."
        )
        require(
            lid_open.min_y > lid_closed.min_y + 0.10,
            "Opening the lid should swing the front edge rearward.",
        )
        require(
            0.5 * (lid_open.max_z + lid_open.min_z)
            > 0.5 * (lid_closed.max_z + lid_closed.min_z) + 0.02,
            "The lid mass should visibly rise as the hood opens.",
        )
        ctx.expect_aabb_overlap("lid", "cart_body", axes="xy", min_overlap=0.06)

    knob_1_rest = part_aabb("knob_1")
    with ctx.pose(knob_1_joint=math.pi):
        knob_1_down = part_aabb("knob_1")
        require(
            knob_1_down.max_z < knob_1_rest.max_z - 0.010,
            "The control marker should rotate away from the upright setting.",
        )
        require(
            knob_1_down.min_z < knob_1_rest.min_z - 0.010,
            "The control marker should sweep downward at the opposite limit.",
        )
    with ctx.pose(knob_1_joint=math.pi / 2.0):
        knob_1_right = part_aabb("knob_1")
        require(
            knob_1_right.max_x > knob_1_rest.max_x + 0.010,
            "The first control knob should visibly sweep sideways.",
        )

    knob_centers_x = []
    control_fascia_front_y = -0.34
    for index in range(1, 5):
        knob_aabb = part_aabb(f"knob_{index}")
        knob_centers_x.append(0.5 * (knob_aabb.min_x + knob_aabb.max_x))
        require(
            knob_aabb.min_y < control_fascia_front_y - 0.02,
            f"knob_{index} should sit proud of the front control fascia.",
        )
        require(
            knob_aabb.max_y > control_fascia_front_y - 0.03,
            f"knob_{index} should remain mounted at the fascia rather than floating far forward.",
        )
        require(
            0.56 < knob_aabb.min_z < 0.66,
            f"knob_{index} should be placed in the central control band.",
        )

    gaps = [knob_centers_x[i + 1] - knob_centers_x[i] for i in range(3)]
    require(
        all(gap > 0.12 for gap in gaps), "Control knobs should be evenly spread across the fascia."
    )
    require(max(gaps) - min(gaps) < 0.03, "Control knob spacing should be visually regular.")

    with ctx.pose(
        lid_hinge=1.20, knob_1_joint=4.0, knob_2_joint=3.1, knob_3_joint=2.2, knob_4_joint=1.0
    ):
        ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=1, overlap_tol=0.004, overlap_volume_tol=0.0)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
