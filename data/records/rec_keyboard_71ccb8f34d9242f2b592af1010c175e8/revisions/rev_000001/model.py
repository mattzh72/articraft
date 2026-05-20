from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq

def make_case():
    case = cq.Workplane("XY").box(0.300, 0.120, 0.018).translate((0, 0, 0.009))
    case = case.faces(">Z").workplane().rect(0.290, 0.110).cutBlind(-0.006)
    return case

def make_keycap(width=0.018, depth=0.018):
    body = cq.Workplane("XY").rect(width, depth).extrude(0.008, taper=5)
    stem = cq.Workplane("XY").rect(0.010, 0.010).extrude(-0.012)
    return body.union(stem)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_keyboard")
    
    case = model.part("case")
    case.visual(
        mesh_from_cadquery(make_case(), "case_mesh"),
        origin=Origin(),
        name="case_visual",
    )
    
    standard_keycap_mesh = mesh_from_cadquery(make_keycap(), "standard_keycap")
    spacebar_mesh = mesh_from_cadquery(make_keycap(width=0.113), "spacebar_keycap")
    
    for r in range(5):
        for c in range(14):
            if r == 4 and 4 <= c <= 9:
                continue
            
            key_name = f"key_{r}_{c}"
            key = model.part(key_name)
            key.visual(
                standard_keycap_mesh,
                origin=Origin(),
                name=f"{key_name}_visual"
            )
            
            x = (c - 6.5) * 0.019
            y = (2 - r) * 0.019
            z = 0.016
            
            model.articulation(
                f"press_{key_name}",
                ArticulationType.PRISMATIC,
                parent=case,
                child=key,
                origin=Origin(xyz=(x, y, z)),
                axis=(0.0, 0.0, -1.0),
                motion_limits=MotionLimits(lower=0.0, upper=0.003, effort=1.0, velocity=1.0)
            )
            
    space_bar = model.part("space_bar")
    space_bar.visual(
        spacebar_mesh,
        origin=Origin(),
        name="space_bar_visual"
    )
    
    model.articulation(
        "press_space_bar",
        ArticulationType.PRISMATIC,
        parent=case,
        child=space_bar,
        origin=Origin(xyz=(0.0, -0.038, 0.016)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.003, effort=2.0, velocity=1.0)
    )
    
    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    case = object_model.get_part("case")
    space_bar = object_model.get_part("space_bar")
    key_0_0 = object_model.get_part("key_0_0")
    
    for r in range(5):
        for c in range(14):
            if r == 4 and 4 <= c <= 9:
                continue
            key_name = f"key_{r}_{c}"
            key = object_model.get_part(key_name)
            ctx.allow_overlap(
                case,
                key,
                reason="Keycap stem is intentionally represented as sliding into the case switch housing."
            )
            
    ctx.allow_overlap(
        case,
        space_bar,
        reason="Spacebar stem is intentionally represented as sliding into the case switch housing."
    )
    
    ctx.expect_within(
        space_bar,
        case,
        axes="xy",
        margin=0.0,
        name="space_bar stays within case footprint"
    )
    
    ctx.expect_within(
        key_0_0,
        case,
        axes="xy",
        margin=0.0,
        name="key_0_0 stays within case footprint"
    )
    
    press_joint = object_model.get_articulation("press_space_bar")
    with ctx.pose({press_joint: 0.003}):
        ctx.expect_within(
            space_bar,
            case,
            axes="xy",
            margin=0.0,
            name="pressed space_bar stays within case footprint"
        )
        
    return ctx.report()

object_model = build_object_model()
